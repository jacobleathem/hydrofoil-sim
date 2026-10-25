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

	void FixedUpdate()
	{
		// 1. Get Input (W for throttle, A/D for steering potentially)
		float throttle = Input.GetAxis("Vertical"); // 0 to 1
		float steer = Input.GetAxis("Horizontal");  // -1 to 1

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