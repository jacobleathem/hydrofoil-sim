using UnityEngine;
// 1. Add this namespace for the New Input System
using UnityEngine.InputSystem;

public class HydrofoilController : MonoBehaviour
{
	[Header("Components")]
	public Rigidbody rb;
	public Transform[] floatPoints;
	public Transform[] foilPoints;

	[Header("Settings")]
	public Transform waterPlane;
	public float buoyancyForce = 15f;
	public float wingLiftFactor = 100f;
	public float enginePower = 10000f;
	public float turnSpeed = 500f;

	[Header("Physics Damping")]
	public float waterDrag = 0.05f; // Keep these low or the boat won't move
	public float airDrag = 0.01f;

	private float waterLevel;

	void Start()
	{
		// COM should be low and centered
		rb.centerOfMass = new Vector3(0, -1f, 0);

		if (waterPlane != null) waterLevel = waterPlane.position.y;
	}

	void Update()
	{
		HandleMovement();

		// 1. BUOYANCY (Hull)
		foreach (var point in floatPoints)
		{
			if (point.position.y < waterLevel)
			{
				float displacement = Mathf.Abs(point.position.y - waterLevel);
				// ForceMode.Acceleration ignores mass, making tuning easier
				rb.AddForceAtPosition(Vector3.up * buoyancyForce * displacement, point.position, ForceMode.Acceleration);

				// Water resistance
				rb.AddForce(-rb.linearVelocity * waterDrag, ForceMode.VelocityChange);
			}
		}

		// 2. HYDROFOIL LIFT (Wings)
		foreach (var foil in foilPoints)
		{
			if (foil.position.y < waterLevel)
			{
				float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
				if (forwardSpeed > 0)
				{
					// Lift is usually speed squared (v^2), making it feel more realistic
					float lift = (forwardSpeed * forwardSpeed) * wingLiftFactor;
					rb.AddForceAtPosition(Vector3.up * lift, foil.position);
				}
			}
		}

		// 3. AIR DRAG
		if (transform.position.y > waterLevel + 0.5f)
		{
			rb.AddForce(-rb.linearVelocity * airDrag, ForceMode.VelocityChange);
		}
	}

	void HandleMovement()
	{
		float move = 0;
		float steer = 0;

		if (Keyboard.current != null)
		{
			if (Keyboard.current.wKey.isPressed) move = 1f;
			if (Keyboard.current.sKey.isPressed) move = -1f;
			if (Keyboard.current.aKey.isPressed) steer = -1f;
			if (Keyboard.current.dKey.isPressed) steer = 1f;
		}

		rb.AddForce(transform.forward * move * enginePower);
		rb.AddTorque(transform.up * steer * turnSpeed);
	}
}