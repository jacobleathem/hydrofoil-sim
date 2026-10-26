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
	public float topSpeed = 20f;
	[Tooltip("Sideways drag as a multiple of forward drag. High = boat carves instead of sliding through turns.")]
	public float lateralDragFactor = 20f;
	[Tooltip("Linear drag on every axis [1/s], so the boat coasts to a stop with the throttle off.")]
	public float linearDrag = 0.3f;

	[Header("Throttle")]
	[Tooltip("Seconds to push the lever from zero to full (or back) while W/S are held. Python mode keeps PilotInput's own, slower rate.")]
	public float leverSweepTime = 1.5f;

	[Header("Turning")]
	[Tooltip("Yaw rate at full steer and full speed [deg/s].")]
	public float maxYawRate = 90f;
	[Tooltip("Share of maxYawRate available when stopped (twin motors can pivot the boat).")]
	[Range(0f, 1f)] public float lowSpeedTurn = 0.5f;
	[Tooltip("How fast the yaw rate follows the stick [1/s].")]
	public float yawResponse = 8f;

	[Header("Banking")]
	[Tooltip("Lean into the turn at full steer and full speed [deg]. Scales with how hard and how fast you turn.")]
	public float maxBankAngle = 12f;
	[Tooltip("How fast the lean follows the turn [1/s].")]
	public float bankResponse = 5f;
	[Tooltip("Share of the lean the onboard camera follows. 0 keeps the horizon level, 1 leans fully with the hull.")]
	[Range(0f, 1f)] public float cameraBankShare = 0.5f;

	[Header("Pilot")]
	[Tooltip("Stick state source. Left empty, the first PilotInput in the scene is used, or one is added here.")]
	public PilotInput pilot;

	// The hull model's long axis is its local +X, not +Z.
	Vector3 Forward => transform.right;

	float yawRate; // deg/s, current turn rate
	float heading; // deg about world up
	float bank;    // deg about the hull's long axis, +ve = leaning right
	Transform cam;
	Quaternion camBaseRotation;

	void Start()
	{
		ResolvePilot();
	}

	void ResolvePilot()
	{
		if (pilot == null) pilot = FindFirstObjectByType<PilotInput>();
		if (pilot == null) pilot = gameObject.AddComponent<PilotInput>();
	}

	void OnEnable()
	{
		if (rb == null) return;

		// Physics never rotates the boat; heading is steered directly in step 4.
		// Freeing yaw alone (pitch/roll locked) made PhysX leak torque into yaw
		// and, with the hull's near-zero inertia, produce NaN angular velocity.
		rb.constraints = RigidbodyConstraints.FreezeRotation;
		rb.angularVelocity = Vector3.zero;
		yawRate = 0f;
		heading = rb.rotation.eulerAngles.y;
		bank = 0f;

		var camera = rb.GetComponentInChildren<Camera>();
		if (camera != null)
		{
			cam = camera.transform;
			camBaseRotation = cam.localRotation;
		}

		ResolvePilot();
		pilot.throttleRate = (PilotInput.SPEED_MAX - PilotInput.SPEED_MIN) / Mathf.Max(0.1f, leverSweepTime);
	}

	void OnDisable()
	{
		if (pilot != null) pilot.throttleRate = PilotInput.SPEED_RATE;
		if (cam != null) cam.localRotation = camBaseRotation;
	}

	void FixedUpdate()
	{
		// 1. Get Input from the shared cockpit stick. Throttle is the lever
		// (holds its position). Steer is the raw stick, not the bank command:
		// the bank ramp is paced for the Python roll PID and made turns feel
		// dead here. Nothing is applied unless the pilot is armed with the
		// deadman held.
		float throttle = pilot.ThrottleEnabled ? pilot.CommandedThrottle01 : 0f; // 0 to 1
		float steer = pilot.ThrottleEnabled ? pilot.SteerInput : 0f;                // -1 to 1

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
		// k is set so full thrust balances both drag terms exactly at topSpeed.
		float k = Mathf.Max(0f, maxThrust - rb.mass * linearDrag * topSpeed) / (topSpeed * topSpeed);
		Vector3 drag = -forward * (k * vForward * Mathf.Abs(vForward))
		               - side * (k * lateralDragFactor * vSide * Mathf.Abs(vSide));
		rb.AddForce(drag);
		rb.AddForce(-v * linearDrag, ForceMode.Acceleration);

		// 4. Yaw. Steer right (+) turns right; reversed when going backwards.
		// The sideways drag above then swings the velocity round with the hull.
		float speedFactor = Mathf.Lerp(lowSpeedTurn, 1f, Mathf.Clamp01(Mathf.Abs(vForward) / topSpeed));
		float targetYaw = steer * maxYawRate * speedFactor * (vForward < -0.1f ? -1f : 1f);
		yawRate = Mathf.Lerp(yawRate, targetYaw, Mathf.Clamp01(yawResponse * Time.fixedDeltaTime));
		heading += yawRate * Time.fixedDeltaTime;

		// 5. Bank into the turn, scaled by the cornering load (speed x turn
		// rate) so a gentle or slow turn barely leans and a hard one at speed
		// leans the full maxBankAngle. Purely visual: physics never tilts it.
		float load = vForward * yawRate / (topSpeed * maxYawRate);
		float targetBank = Mathf.Clamp(load, -1f, 1f) * maxBankAngle;
		bank = Mathf.Lerp(bank, targetBank, Mathf.Clamp01(bankResponse * Time.fixedDeltaTime));

		// The long axis is local +X; a negative turn about it drops the right side.
		rb.MoveRotation(Quaternion.Euler(0f, heading, 0f) * Quaternion.AngleAxis(-bank, Vector3.right));
		if (cam != null)
			cam.localRotation = Quaternion.AngleAxis(bank * (1f - cameraBankShare), Vector3.right) * camBaseRotation;

		// 6. Calculate Forward Velocity for Simulink
		currentSpeed = vForward;
	}
}