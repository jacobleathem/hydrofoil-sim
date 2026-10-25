using UnityEngine;

public class HydrofoilElement : MonoBehaviour
{
	public Rigidbody rb;
	public float waterDensity = 1025f; // Seawater density
	public float foilArea = 2.0f;      // Size of the underwater wing
	public AnimationCurve liftCurve;    // AoA vs Lift Coefficient

	[Header("Water Interaction")]
	public float waterLevel = 0f;

	void FixedUpdate()
	{
		// 1. Only apply lift if the foil is actually underwater
		if (transform.position.y < waterLevel)
		{

			// 2. Calculate Relative Velocity
			Vector3 relativeVelocity = transform.InverseTransformDirection(rb.GetPointVelocity(transform.position));
			float speedSquared = relativeVelocity.sqrMagnitude;

			// 3. Calculate Angle of Attack (AoA)
			// Assuming the foil's forward is transform.forward
			float aoa = Mathf.Atan2(-relativeVelocity.y, relativeVelocity.z) * Mathf.Rad2Deg;

			// 4. Calculate Lift Force: L = 0.5 * rho * v^2 * S * Cl
			float liftCoef = liftCurve.Evaluate(aoa);
			float liftForceMagnitude = 0.5f * waterDensity * speedSquared * foilArea * liftCoef;

			// 5. Apply Force perpendicular to the foil's travel direction
			Vector3 liftDir = transform.up;
			rb.AddForceAtPosition(liftDir * liftForceMagnitude, transform.position);

			// 6. Add Drag (to prevent infinite speed)
			float dragForceMagnitude = 0.5f * waterDensity * speedSquared * foilArea * 0.05f; // 0.05 is a base drag coef
			rb.AddForceAtPosition(-rb.linearVelocity.normalized * dragForceMagnitude, transform.position);
		}
	}
}
