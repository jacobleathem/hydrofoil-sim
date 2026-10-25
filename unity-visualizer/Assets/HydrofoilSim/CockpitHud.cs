using UnityEngine;

/// <summary>
/// Driver-training HUD: arming state, deadman, and the pilot's commands from
/// PilotInput. Top-left of the Game view, so it doesn't overlap
/// PythonSimDriver's flap readout in the top-right.
///
/// IMGUI overlay, not a Canvas — same reasoning as PythonSimDriver's HUD:
/// no UI GameObjects to keep in sync. Created automatically on scene load if
/// the scene doesn't already have one.
/// </summary>
public class CockpitHud : MonoBehaviour
{
    public bool showHud = true;
    [Tooltip("Left empty, the first PilotInput in the scene is used.")]
    public PilotInput pilot;

    static readonly Color ArmedColor = new Color(0.3f, 1f, 0.4f);
    static readonly Color DisarmedColor = new Color(1f, 0.35f, 0.3f);
    static readonly Color DimColor = new Color(0.75f, 0.75f, 0.75f);

    private GUIStyle _label, _title, _big, _hint;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void AutoCreate()
    {
        if (FindFirstObjectByType<CockpitHud>() == null)
            new GameObject("CockpitHud").AddComponent<CockpitHud>();
    }

    void OnGUI()
    {
        if (!showHud) return;
        // PilotInput may be added at runtime by a driver's Start(), so keep looking.
        if (pilot == null) pilot = FindFirstObjectByType<PilotInput>();
        EnsureStyles();

        const float w = 260f, margin = 10f;
        GUILayout.BeginArea(new Rect(margin, margin, w, Screen.height - 2 * margin));
        GUILayout.BeginVertical(GUI.skin.box);
        DrawPilotPanel();
        GUILayout.EndVertical();
        GUILayout.EndArea();
    }

    void DrawPilotPanel()
    {
        if (pilot == null)
        {
            GUILayout.Label("No PilotInput in scene", _label);
            return;
        }

        bool live = pilot.ThrottleEnabled;
        _big.normal.textColor = live ? ArmedColor : DisarmedColor;
        GUILayout.Label(live ? "THROTTLE LIVE" : (pilot.Armed ? "ARMED - THROTTLE CUT" : "DISARMED"), _big);
        GUILayout.Label(pilot.Status, _label);

        GUILayout.Space(4);
        GUILayout.Label("Pilot", _title);
        Row("Armed", pilot.requireArming ? (pilot.Armed ? "yes" : "no") : "not required");
        Row("Deadman", pilot.requireDeadman ? (pilot.DeadmanHeld ? "held" : "released") : "not required");
        Row("Throttle lever", $"{pilot.TargetSpeed,5:F1} m/s  ({pilot.Throttle01 * 100f,3:F0}%)");
        Row("Commanded", $"{pilot.CommandedSpeed,5:F1} m/s");
        Row("Bank cmd", $"{pilot.PhiRef,5:F1}°");
        Row("Height cmd", $"{pilot.FlyingHeight,5:F2} m");

        GUILayout.Space(4);
        GUILayout.Label("Hold Space (deadman) + E to arm\nW/S throttle   A/D bank   ↑/↓ height\nPad: RB deadman, Start arm, sticks", _hint);
    }

    void Row(string name, string value)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(name, _label, GUILayout.Width(110));
        GUILayout.Label(value, _label);
        GUILayout.EndHorizontal();
    }

    void EnsureStyles()
    {
        if (_label != null) return;
        _label = new GUIStyle(GUI.skin.label) { fontSize = 14, normal = { textColor = Color.white } };
        _title = new GUIStyle(_label) { fontStyle = FontStyle.Bold };
        _big = new GUIStyle(_label) { fontSize = 20, fontStyle = FontStyle.Bold };
        _hint = new GUIStyle(_label) { fontSize = 12, normal = { textColor = DimColor } };
    }
}
