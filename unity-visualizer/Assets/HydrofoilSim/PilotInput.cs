using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Single owner of the pilot's stick state, shared by both sim modes:
/// PythonSimDriver sends it to the Python plant, TwinMotorController uses it
/// for Unity's arcade physics. Keeping it in one place means the cockpit
/// feels the same whichever mode is driving the boat.
///
/// Keyboard:  W/S throttle, A/D bank, Up/Down flying height,
///            E arm/disarm, hold Space = deadman
/// Gamepad:   left stick Y throttle, right stick X bank, D-pad up/down height,
///            Start arm/disarm, hold right shoulder = deadman
///
/// Height, throttle and bank are targets for the autopilot (heave, surge,
/// roll PIDs in Python), not direct force commands.
///
/// Safety: throttle only reaches the boat while armed and the deadman is held.
/// Arming needs the throttle lever at zero (and the deadman held), so the
/// boat can't lurch off the moment it's armed. Letting go of the deadman
/// disarms, like a kill-cord lanyard. Both checks can be switched off in the
/// Inspector for bench testing.
/// </summary>
public class PilotInput : MonoBehaviour
{
    // Height/speed match dashboard.py's manualControlTick() feel.
    public const float HEIGHT_RATE = 0.15f;
    public const float HEIGHT_MIN = -0.3f;
    public const float HEIGHT_MAX = 1.0f;
    public const float SPEED_RATE = 1.0f;      // m/s of target per second held
    public const float SPEED_MIN = 0.0f;       // 0 = no lift possible — this is how you feel gravity win
    public const float SPEED_MAX = 10.0f;
    // Bank range is deliberately tight — the roll PID (server-side) is only
    // verified stable for a commanded bank up to ~10 deg; 12 deg reliably
    // diverges (see control_uplink.py's BANK_MIN/MAX docstring). Don't widen
    // this without re-verifying there too.
    public const float BANK_RATE = 6.0f;       // deg of target per second held
    public const float BANK_MIN = -8.0f;
    public const float BANK_MAX = 8.0f;
    public const float BANK_RETURN_RATE = 10.0f;  // deg/s bank relaxes toward 0 when A/D released
    public const float ARM_MAX_SPEED = 0.05f;  // lever must be at (about) zero to arm

    [Header("Safety")]
    [Tooltip("Throttle is held at zero until the pilot arms (E / gamepad Start).")]
    public bool requireArming = true;
    [Tooltip("Throttle needs the deadman held (Space / gamepad right shoulder); releasing it disarms.")]
    public bool requireDeadman = true;

    [Header("Gamepad")]
    [Tooltip("Stick values below this are treated as centred.")]
    [Range(0f, 0.5f)] public float stickDeadzone = 0.15f;

    [Header("Pilot State (read-only)")]
    [SerializeField] private float _flyingHeight = 0.55f;
    [SerializeField] private float _targetSpeed = 0.0f;
    [SerializeField] private float _phiRef = 0.0f;
    [SerializeField] private bool _armed;
    [SerializeField] private bool _deadmanHeld;
    [SerializeField] private string _status = "Disarmed";

    /// <summary>Commanded ride height for the heave PID [m].</summary>
    public float FlyingHeight => _flyingHeight;
    /// <summary>Throttle lever position, as the surge controller's target speed [m/s].</summary>
    public float TargetSpeed => _targetSpeed;
    /// <summary>Throttle lever position normalised to 0..1.</summary>
    public float Throttle01 => Mathf.InverseLerp(SPEED_MIN, SPEED_MAX, _targetSpeed);
    /// <summary>Commanded bank angle [deg], +ve = rolled right (see pid.py).</summary>
    public float PhiRef => _phiRef;
    /// <summary>Bank command normalised to -1..1.</summary>
    public float Steer01 => _phiRef / BANK_MAX;

    public bool Armed => _armed;
    public bool DeadmanHeld => _deadmanHeld;
    /// <summary>Short human-readable arming state, e.g. why the last arm attempt was refused.</summary>
    public string Status => _status;
    /// <summary>True when throttle is allowed through to the boat.</summary>
    public bool ThrottleEnabled => (!requireArming || _armed) && (!requireDeadman || _deadmanHeld);
    /// <summary>Speed actually commanded: the lever, or zero when throttle is cut [m/s].</summary>
    public float CommandedSpeed => ThrottleEnabled ? _targetSpeed : 0f;
    /// <summary>CommandedSpeed normalised to 0..1.</summary>
    public float CommandedThrottle01 => ThrottleEnabled ? Throttle01 : 0f;

    void Update()
    {
        var kb = Keyboard.current;
        var pad = Gamepad.current;
        float dt = Time.deltaTime;

        UpdateSafety(kb, pad);

        // Climb/dive: holds like a lever (no spring-back).
        float heightAxis = 0f;
        if (kb != null) heightAxis += (kb.upArrowKey.isPressed ? 1f : 0f) - (kb.downArrowKey.isPressed ? 1f : 0f);
        if (pad != null) heightAxis += (pad.dpad.up.isPressed ? 1f : 0f) - (pad.dpad.down.isPressed ? 1f : 0f);
        heightAxis = Mathf.Clamp(heightAxis, -1f, 1f);
        _flyingHeight = Mathf.Clamp(_flyingHeight + HEIGHT_RATE * heightAxis * dt, HEIGHT_MIN, HEIGHT_MAX);

        // Throttle: holds like a real lever (no spring-back to a default) — at
        // SPEED_MIN there's no lift available at all, so the heave PID can't hold
        // height and the hull sinks back into the water under gravity.
        float throttleAxis = 0f;
        if (kb != null) throttleAxis += (kb.wKey.isPressed ? 1f : 0f) - (kb.sKey.isPressed ? 1f : 0f);
        if (pad != null) throttleAxis += ApplyDeadzone(pad.leftStick.y.ReadValue());
        throttleAxis = Mathf.Clamp(throttleAxis, -1f, 1f);
        _targetSpeed = Mathf.Clamp(_targetSpeed + SPEED_RATE * throttleAxis * dt, SPEED_MIN, SPEED_MAX);

        // Bank: spring-centred. On the gamepad the stick sets the bank directly
        // (full deflection = BANK_MAX); on the keyboard A/D ramp it and it relaxes
        // back to 0 when released, like a self-centering stick.
        float stickBank = pad != null ? ApplyDeadzone(pad.rightStick.x.ReadValue()) : 0f;
        bool bankRight = kb != null && kb.dKey.isPressed;
        bool bankLeft  = kb != null && kb.aKey.isPressed;
        if (stickBank != 0f)
        {
            _phiRef = Mathf.Clamp(stickBank * BANK_MAX, BANK_MIN, BANK_MAX);
        }
        else if (bankRight != bankLeft)
        {
            _phiRef = Mathf.Clamp(_phiRef + BANK_RATE * dt * (bankRight ? 1f : -1f), BANK_MIN, BANK_MAX);
        }
        else
        {
            _phiRef = Mathf.MoveTowards(_phiRef, 0f, BANK_RETURN_RATE * dt);
        }
    }

    void UpdateSafety(Keyboard kb, Gamepad pad)
    {
        _deadmanHeld = (kb != null && kb.spaceKey.isPressed) || (pad != null && pad.rightShoulder.isPressed);

        if (_armed && requireDeadman && !_deadmanHeld)
        {
            _armed = false;
            _status = "Disarmed: deadman released";
        }

        bool armToggle = (kb != null && kb.eKey.wasPressedThisFrame) || (pad != null && pad.startButton.wasPressedThisFrame);
        if (!armToggle) return;

        if (_armed)
        {
            _armed = false;
            _status = "Disarmed";
        }
        else if (_targetSpeed > ARM_MAX_SPEED)
        {
            _status = "Can't arm: bring throttle to zero";
        }
        else if (requireDeadman && !_deadmanHeld)
        {
            _status = "Can't arm: hold the deadman";
        }
        else
        {
            _armed = true;
            _status = "Armed";
        }
    }

    float ApplyDeadzone(float v)
    {
        if (Mathf.Abs(v) < stickDeadzone) return 0f;
        return Mathf.Sign(v) * Mathf.InverseLerp(stickDeadzone, 1f, Mathf.Abs(v));
    }
}
