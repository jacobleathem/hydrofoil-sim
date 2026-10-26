using UnityEngine;
using UnityEngine.InputSystem;
public class TwinMotorController : MonoBehaviour
{
	public Rigidbody rb;

	[Header("Motor Positions")]
	// Not used for thrust while pitch and roll are frozen; see FixedUpdate.
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

	[Header("Turning")]
	[Tooltip("Yaw rate at full steer and full speed [deg/s].")]
	public float maxYawRate = 60f;
	[Tooltip("Share of maxYawRate available when stopped (twin motors can pivot the boat).")]
	[Range(0f, 1f)] public float lowSpeedTurn = 0.35f;
	[Tooltip("How fast the yaw rate follows the stick [1/s].")]
	public float yawResponse = 4f;
	[Tooltip("Hull box (local X length, Y height, Z beam) [m] used for the inertia tensor.")]
	public Vector3 hullSize = new Vector3(1.2f, 0.3f, 0.8f);

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

	void OnEnable()
	{
		if (rb == null) return;

		// The scene freezes all rotation. Leave yaw free so the boat can turn;
		// pitch and roll stay frozen for now.
		rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

		// The inertia Unity derives from the hull's few tiny colliders is near
		// zero and tilted about 1.5 deg, which couples any stray torque into yaw.
		// Use a solid box instead.
		Vector3 s = hullSize;
		rb.inertiaTensor = rb.mass / 12f * new Vector3(s.y * s.y + s.z * s.z, s.x * s.x + s.z * s.z, s.x * s.x + s.y * s.y);
		rb.inertiaTensorRotation = Quaternion.identity;
	}

	void FixedUpdate()
	{
		// 1. Get Input from the shared cockpit stick. Throttle is the lever
		// (holds its position), steer follows the bank command. Nothing is
		// applied unless the pilot is armed with the deadman held.
		float throttle = pilot.ThrottleEnabled ? pilot.CommandedThrottle01 : 0f; // 0 to 1
		float steer = pilot.ThrottleEnabled ? pilot.Steer01 : 0f;                 // -1 to 1

		// 2. Thrust, through the centre of mass. Pitch and roll are frozen, and
		// the motors sit a few mm off-centre, so pushing at their position only
		// added a yaw drift. Turning is handled in step 4.
		Vector3 forward = Forward;
		if (throttle > 0f)
			rb.AddForce(forward * throttle * maxThrust);

		// 3. Water drag. Quadratic along the hull, much stronger sideways.
		Vector3 v = rb.linearVelocity;
		float vForward = Vector3.Dot(v, forward);
		Vector3 side = Vector3.Cross(Vector3.up, forward).normalized;
		float vSide = Vector3.Dot(v, side);
		float k = maxThrust / (topSpeed * topSpeed);
		Vector3 drag = -forward * (k * vForward * Mathf.Abs(vForward))
		               - side * (k * lateralDragFactor * vSide * Mathf.Abs(vSide));
		rb.AddForce(drag);
		rb.AddForce(-v * linearDrag, ForceMode.Acceleration);

		// 4. Yaw. Steer right (+) turns right; reversed when going backwards.
		float speedFactor = Mathf.Lerp(lowSpeedTurn, 1f, Mathf.Clamp01(Mathf.Abs(vForward) / topSpeed));
		float targetYaw = steer * maxYawRate * Mathf.Deg2Rad * speedFactor * (vForward < -0.1f ? -1f : 1f);
		Vector3 w = rb.angularVelocity;
		w.y = Mathf.Lerp(w.y, targetYaw, Mathf.Clamp01(yawResponse * Time.fixedDeltaTime));
		rb.angularVelocity = w;

		// 5. Calculate Forward Velocity for Simulink
		currentSpeed = vForward;
	}
}