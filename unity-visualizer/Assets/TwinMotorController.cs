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

	[Header("Pilot")]
	[Tooltip("Stick state source. Left empty, the first PilotInput in the scene is used, or one is added here.")]
	public PilotInput pilot;

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
			rb.AddForceAtPosition(transform.right * leftThrust, leftMotorPos.position);

		if (rightThrust > 0)
			rb.AddForceAtPosition(transform.right * rightThrust, rightMotorPos.position);

		// 4. Calculate Forward Velocity for Simulink
		currentSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
	}
}