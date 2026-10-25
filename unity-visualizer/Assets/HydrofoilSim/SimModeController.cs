using UnityEngine;

/// <summary>
/// Arbitrates between the hand-built Unity arcade physics (Rigidbody + Floater +
/// TwinMotorController) and PythonSimDriver directly writing the ship transform.
/// Both were live at once before this existed, fighting over the same Rigidbody.
/// </summary>
public class SimModeController : MonoBehaviour
{
    public enum Mode { Arcade, PythonDriven }

    [SerializeField] private Mode mode = Mode.PythonDriven;
    public Mode CurrentMode => mode;

    private Rigidbody _rb;
    private Floater[] _floaters;
    private TwinMotorController _motor;

    void Awake()
    {
        _rb = GetComponent<Rigidbody>();
        _floaters = GetComponentsInChildren<Floater>(true);
        _motor = GetComponentInChildren<TwinMotorController>(true);
        Apply();
    }

    public void SetMode(Mode newMode)
    {
        mode = newMode;
        Apply();
    }

    private void Apply()
    {
        bool pythonDriven = mode == Mode.PythonDriven;

        if (_rb != null) _rb.isKinematic = pythonDriven;
        foreach (var f in _floaters) f.enabled = !pythonDriven;
        if (_motor != null) _motor.enabled = !pythonDriven;
    }

#if UNITY_EDITOR
    void OnValidate()
    {
        if (_floaters == null) return; // not yet Awake-initialized in edit mode
        Apply();
    }
#endif
}
