using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// Drop-in replacement for SimulinkTCPReceiver.
/// Receives JSON state from the Python hydrofoil sim over UDP.
///
/// Setup:
///   1. Remove SimulinkTCPReceiver from your ship GameObject
///   2. Add this script to the same GameObject
///   3. Assign the same shipTransform / leftFoil / rightFoil references
///   4. Press Play, then run: python3 -m hydrofoil.sim_realtime  (in WSL2)
///
/// Python → Unity coordinate conversion (NED → Unity left-handed Y-up):
///   NED x (forward) → Unity +Z
///   NED y (right)   → Unity +X
///   NED z (down)    → Unity -Y
/// </summary>
public class PythonSimDriver : MonoBehaviour
{
    [Header("Transforms (same as SimulinkTCPReceiver)")]
    public Transform shipTransform;
    public Transform leftFoil;
    public Transform rightFoil;
    public Transform elevatorFoil;   // optional rear elevator

    [Header("Network")]
    public int port = 5005;

    [Header("Pilot Input (climb/dive, sent back to Python)")]
    [Tooltip("Send the PilotInput stick state (height, throttle, bank) back to Python")]
    public bool pilotInputEnabled = true;
    [Tooltip("Where the stick state comes from. Left empty, the first PilotInput in the scene " +
             "is used, or one is added to this GameObject.")]
    public PilotInput pilot;
    public string pythonHost = "127.0.0.1";
    public int pilotPort = 5006;

    [Header("Auto-Launch sim_realtime.py in WSL")]
    [Tooltip("Launch the Python sim automatically when Play starts, in the named WSL distro — removes the manual 'open a WSL terminal and type the right command' step entirely.")]
    public bool autoLaunchSim = true;
    public string wslDistro = "Ubuntu";
    [Tooltip("Command run inside `wsl -d <distro> -e bash -lc \"<this>\"`. --host 127.0.0.1 " +
             "is required here — without it, Python auto-detects the WSL gateway IP, which " +
             "does not route correctly under mirrored networking (verified: packets silently vanish).")]
    public string simCommand = "cd /home/jleathem/code/unity && python3 -u -m hydrofoil.sim_realtime --host 127.0.0.1 --duration 86400";

    [Header("Scene Offset")]
    [Tooltip("Y position of the water surface in your scene")]
    public float waterSurfaceY = 0f;
    [Tooltip("Yaw correction applied on top of the sim's psi, to align the hull mesh's " +
             "modeled forward axis with the physics' forward axis (Unity +Z). Unity's Y-rotation " +
             "is clockwise from above, so a 90 degree CCW correction is -90.")]
    public float modelYawOffsetDeg = -90f;

    [Header("Debug")]
    public bool showGizmos = true;

    [Header("HUD")]
    [Tooltip("Draw the flap-angle readout in the top-right corner of the Game view.")]
    public bool showFlapHud = true;

    // ── Live state (read-only in Inspector) ───────────────────────────────
    [Header("Live State (read-only)")]
    [SerializeField] private float _time;
    [SerializeField] private float _depth;
    [SerializeField] private float _pitchDeg;
    [SerializeField] private float _rollDeg;
    [SerializeField] private float _speed;
    [SerializeField] private float _deltaMain;
    [SerializeField] private float _deltaElev;
    [SerializeField] private float _deltaRoll;
    [SerializeField] private float _thrust;
    [SerializeField] private float _draft;
    [SerializeField] private bool  _foilborne;
    [SerializeField] private float _waveElev;
    [SerializeField] private int   _packetsReceived;

    // ── Threading ─────────────────────────────────────────────────────────
    private UdpClient       _udp;
    private Thread          _thread;
    private volatile bool   _running;
    private SimPacket       _latest;
    private volatile bool   _hasNew;
    private readonly object _lock = new object();

    // ── Packet struct ─────────────────────────────────────────────────────
    [Serializable]
    private class SimPacket
    {
        public float t;
        public float x, y, z;           // NED position [m]
        public float phi, theta, psi;   // ZYX Euler [rad]
        public float u, v, w;           // body velocity [m/s]
        public float d_main, d_elev, d_roll;  // foil deflections [deg]
        public float thrust;            // commanded thrust [N]
        public float draft;             // hull draft below local water surface [m]
        public bool  foilborne;         // draft <= 0, hull clear of the water
        public float wave_elev;         // wave surface elevation at the vehicle's position [m]
    }

    // Climb/dive + throttle + bank pilot stick (from PilotInput), sent back to
    // Python. All three are targets fed to the existing PIDs (heave, surge,
    // roll) running server-side — this is a stick into an autopilot, not
    // direct force control.
    private const float PILOT_SEND_INTERVAL = 0.05f;   // ~20Hz, matches the dashboard's throttle
    private float _pilotSendTimer;

    private System.Diagnostics.Process _simProcess;

    // ── Lifecycle ─────────────────────────────────────────────────────────

    void Start()
    {
        // Without this, the Editor's Player Loop can stall to ~1 frame total
        // while the window lacks OS focus (observed directly: frameCount stuck
        // at 1 after 60+ real seconds) — the background UDP thread keeps
        // receiving fine, but Update() never runs to consume it, so nothing
        // ever visibly moves. Keeps this driving Unity correctly regardless
        // of whether the window has focus.
        Application.runInBackground = true;

        if (shipTransform != null)
        {
            var modeController = shipTransform.GetComponent<SimModeController>();
            if (modeController != null) modeController.SetMode(SimModeController.Mode.PythonDriven);
        }

        if (pilot == null) pilot = FindFirstObjectByType<PilotInput>();
        if (pilot == null) pilot = gameObject.AddComponent<PilotInput>();

        _udp     = new UdpClient(new IPEndPoint(IPAddress.Any, port));
        _udp.Client.ReceiveTimeout = 500;
        _running = true;
        _thread  = new Thread(ReceiveLoop) { IsBackground = true, Name = "PythonSimDriver" };
        _thread.Start();
        Debug.Log($"[PythonSimDriver] Listening on UDP port {port}. Run: python3 -m hydrofoil.sim_realtime");

        if (autoLaunchSim) LaunchSim();
    }

    void LaunchSim()
    {
        if (_simProcess != null && !_simProcess.HasExited) return;   // already running, don't double-launch

        try
        {
            var psi = new System.Diagnostics.ProcessStartInfo
            {
                FileName = "wsl.exe",
                Arguments = $"-d {wslDistro} -e bash -lc \"{simCommand}\"",
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
            };
            _simProcess = new System.Diagnostics.Process { StartInfo = psi, EnableRaisingEvents = true };
            _simProcess.OutputDataReceived += (s, e) => { if (e.Data != null) Debug.Log($"[sim_realtime] {e.Data}"); };
            _simProcess.ErrorDataReceived  += (s, e) => { if (e.Data != null) Debug.LogWarning($"[sim_realtime] {e.Data}"); };
            _simProcess.Start();
            _simProcess.BeginOutputReadLine();
            _simProcess.BeginErrorReadLine();
            Debug.Log($"[PythonSimDriver] Auto-launched sim_realtime.py in WSL distro '{wslDistro}'. Its output appears above as [sim_realtime] lines.");
        }
        catch (Exception e)
        {
            Debug.LogError($"[PythonSimDriver] Failed to auto-launch WSL sim: {e.Message}. "
                + "Falling back to manual: run this yourself —\n"
                + $"wsl -d {wslDistro} -e bash -lc \"{simCommand}\"");
        }
    }

    void Update()
    {
        if (_hasNew)
        {
            SimPacket p;
            lock (_lock) { p = _latest; _hasNew = false; }

            ApplyToScene(p);
            UpdateInspector(p);
            _packetsReceived++;
        }

        if (pilotInputEnabled) SendPilotInput();
    }

    void SendPilotInput()
    {
        _pilotSendTimer += Time.deltaTime;
        if (_pilotSendTimer < PILOT_SEND_INTERVAL) return;
        _pilotSendTimer = 0f;

        try
        {
            var inv = System.Globalization.CultureInfo.InvariantCulture;
            string json = $"{{\"flying_height\":{pilot.FlyingHeight.ToString(inv)}," +
                          $"\"target_speed\":{pilot.TargetSpeed.ToString(inv)}," +
                          $"\"phi_ref\":{pilot.PhiRef.ToString(inv)}}}";
            byte[] data = Encoding.UTF8.GetBytes(json);
            _udp.Send(data, data.Length, pythonHost, pilotPort);
        }
        catch (Exception e) { Debug.LogWarning($"[PythonSimDriver] Pilot input send failed: {e.Message}"); }
    }

    void OnDestroy()
    {
        _running = false;
        _udp?.Close();
        _thread?.Join(800);

        if (_simProcess != null)
        {
            try { if (!_simProcess.HasExited) _simProcess.Kill(); }
            catch (Exception e) { Debug.LogWarning($"[PythonSimDriver] Failed to stop local wsl.exe process: {e.Message}"); }
            finally { _simProcess.Dispose(); _simProcess = null; }

            // Killing wsl.exe does NOT reliably kill the process it started inside
            // the WSL instance (verified: the Linux-side python survived it) — the
            // WSL VM persists independently of any one wsl.exe invocation. Target
            // it explicitly by name inside WSL too, fire-and-forget.
            if (autoLaunchSim)
            {
                try
                {
                    var killPsi = new System.Diagnostics.ProcessStartInfo
                    {
                        FileName = "wsl.exe",
                        Arguments = $"-d {wslDistro} -e bash -lc \"pkill -f hydrofoil.sim_realtime\"",
                        UseShellExecute = false,
                        CreateNoWindow = true,
                    };
                    System.Diagnostics.Process.Start(killPsi);
                }
                catch (Exception e) { Debug.LogWarning($"[PythonSimDriver] Failed to pkill sim_realtime.py in WSL: {e.Message}"); }
            }
        }
    }

    // ── UDP receive loop ──────────────────────────────────────────────────

    void ReceiveLoop()
    {
        var ep = new IPEndPoint(IPAddress.Any, 0);
        while (_running)
        {
            try
            {
                byte[] data  = _udp.Receive(ref ep);
                string json  = Encoding.UTF8.GetString(data);
                var    pkt   = JsonUtility.FromJson<SimPacket>(json);
                if (pkt == null) continue;
                lock (_lock) { _latest = pkt; _hasNew = true; }
            }
            catch (SocketException) { /* timeout — check _running */ }
            catch (Exception e) { Debug.LogWarning($"[PythonSimDriver] {e.Message}"); }
        }
    }

    // ── Scene application ─────────────────────────────────────────────────

    void ApplyToScene(SimPacket p)
    {
        if (shipTransform == null) return;

        // NED → Unity: x=NED.y, y=-(NED.z) + surfaceY, z=NED.x
        Vector3 unityPos = new Vector3(p.y, -p.z + waterSurfaceY, p.x);
        shipTransform.position = unityPos;

        // Euler angles: pitch θ (+nose-up NED) → Unity -X; yaw ψ → +Y; roll φ → +Z.
        // modelYawOffsetDeg corrects for the hull mesh's authored forward axis.
        shipTransform.rotation = Quaternion.Euler(
            -p.theta * Mathf.Rad2Deg,
             p.psi   * Mathf.Rad2Deg + modelYawOffsetDeg,
             p.phi   * Mathf.Rad2Deg);

        // Foil flap rotations. d_main is the common (heave) deflection, d_roll is the
        // differential added on top so the two flaps counter-rotate for roll/turn authority.
        if (leftFoil  != null) leftFoil.localRotation  = Quaternion.Euler(-(p.d_main - p.d_roll), 0, 0);
        if (rightFoil != null) rightFoil.localRotation = Quaternion.Euler(-(p.d_main + p.d_roll), 0, 0);
        if (elevatorFoil != null) elevatorFoil.localRotation = Quaternion.Euler(-p.d_elev, 0, 0);
    }

    void UpdateInspector(SimPacket p)
    {
        _time      = p.t;
        _depth     = p.z;
        _pitchDeg  = p.theta * Mathf.Rad2Deg;
        _rollDeg   = p.phi   * Mathf.Rad2Deg;
        _speed     = p.u;
        _deltaMain = p.d_main;
        _deltaElev = p.d_elev;
        _deltaRoll = p.d_roll;
        _thrust    = p.thrust;
        _draft     = p.draft;
        _foilborne = p.foilborne;
        _waveElev  = p.wave_elev;
    }

    // ── Gizmos ────────────────────────────────────────────────────────────

    void OnDrawGizmos()
    {
        if (!showGizmos || shipTransform == null) return;
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(shipTransform.position, 0.15f);
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(shipTransform.position,
            shipTransform.position + shipTransform.forward * 0.8f);
    }

    // ── HUD ───────────────────────────────────────────────────────────────
    // IMGUI overlay, not a Canvas — keeps this readout working without any
    // scene wiring (no UI GameObjects/prefab dependency to keep in sync).

    private GUIStyle _hudLabelStyle;
    private GUIStyle _hudTitleStyle;

    void OnGUI()
    {
        if (!showFlapHud) return;

        if (_hudLabelStyle == null)
        {
            _hudLabelStyle = new GUIStyle(GUI.skin.label) { fontSize = 14, normal = { textColor = Color.white } };
            _hudTitleStyle = new GUIStyle(_hudLabelStyle) { fontStyle = FontStyle.Bold };
        }

        const float w = 190f, h = 100f, margin = 10f;
        Rect box = new Rect(Screen.width - w - margin, margin, w, h);
        GUI.Box(box, GUIContent.none);

        float deltaLeft  = _deltaMain - _deltaRoll;
        float deltaRight = _deltaMain + _deltaRoll;

        GUILayout.BeginArea(new Rect(box.x + 8, box.y + 4, w - 16, h - 8));
        GUILayout.Label("Flap Angles", _hudTitleStyle);
        GUILayout.Label($"Front Left:   {deltaLeft,6:F1}°", _hudLabelStyle);
        GUILayout.Label($"Front Right:  {deltaRight,6:F1}°", _hudLabelStyle);
        GUILayout.Label($"Back Wing:    {_deltaElev,6:F1}°", _hudLabelStyle);
        GUILayout.EndArea();
    }
}
