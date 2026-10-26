using UnityEngine;
using UnityEngine.InputSystem;
public class TwinMotorController : MonoBehaviour
{
	public Rigidbody rb;

	[Header("Motor Positions")]
	public Transform leftMotorPos;  // Drag 'LeftMotorPos' here
	public Transform rightMotorPos; // Drag 'RightMotorPos' here

	[Header("Settings")]
	public float maxThrust = 5000f; // N change num
	public float currentSpeed;      // Send this to Simulink!

	[Header("Water Drag")]
	[Tooltip("Speed reached at full throttle [m/s]. Forward drag is quadratic and tuned so it balances maxThrust here.")]
	public float topSpeed = 10f;
	[Tooltip("Sideways drag as a multiple of forward drag. High = boat carves instead of sliding through turns.")]
	public float lateralDragFactor = 20f;
	[Tooltip("Linear drag on every axis [1/s], so the boat coasts to a stop with the throttle off.")]
	public float linearDrag = 0.3f;

	[Header("Pilot")]
	[Tooltip("Stick state source. Left empty, the first PilotInput in the scene is used, or one is added here.")]
	public PilotInput pilot;

	// The hull model's long axis is its local +X, not +Z.
	Vector3 Forward => transform.right;

	void Start()
	{
		if (pilot == null) pilot = FindFirstObjectByType<PilotInput>();
		if (pilot == null) pilot = gameObject.AddComponent<PilotInput>();
	}

	void FixedUpdate()
	{
		// 1. Get Input from the shared cockpit stick. Throttle is the lever
		// (holds its position), steer follows the bank command. Nothing is
		// applied unless the pilot is armed with the deadman held.
		float throttle = pilot.ThrottleEnabled ? pilot.CommandedThrottle01 : 0f; // 0 to 1
		float steer = pilot.ThrottleEnabled ? pilot.Steer01 : 0f;                 // -1 to 1

		// 2. Calculate Thrust for each motor
		// Simple "Tank Steering" logic: 
		// Turning Right (D) -> Left motor pushes harder
		float leftThrust = Mathf.Clamp01(throttle + steer * 0.5f) * maxThrust;
		float rightThrust = Mathf.Clamp01(throttle - steer * 0.5f) * maxThrust;

		// 3. Apply Force at specific POSITIONS
		// This creates realistic Torque (Pitch up + Yaw)
		if (leftThrust > 0)
			rb.AddForceAtPosition(Forward * leftThrust, leftMotorPos.position);

		if (rightThrust > 0)
			rb.AddForceAtPosition(Forward * rightThrust, rightMotorPos.position);

		// 4. Water drag. Quadratic along the hull, much stronger sideways.
		Vector3 forward = Forward;
		Vector3 v = rb.linearVelocity;
		float vForward = Vector3.Dot(v, forward);
		Vector3 side = Vector3.Cross(Vector3.up, forward).normalized;
		float vSide = Vector3.Dot(v, side);
		float k = maxThrust / (topSpeed * topSpeed);
		Vector3 drag = -forward * (k * vForward * Mathf.Abs(vForward))
		               - side * (k * lateralDragFactor * vSide * Mathf.Abs(vSide));
		rb.AddForce(drag);
		rb.AddForce(-v * linearDrag, ForceMode.Acceleration);

		// 5. Calculate Forward Velocity for Simulink
		currentSpeed = vForward;
	}
}