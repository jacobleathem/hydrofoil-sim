using UnityEngine;

public class Floater : MonoBehaviour
{
    public Rigidbody foilBody;
    public Transform waterPlane;
    public int floaterCount;
	public float floatingThreshold = 2.0f;
	public float waterDensity = 0.125f;
	public float waterDrag = 0.025f;
	public float waterAngularDrag = 0.025f;
	private void FixedUpdate()
    {
		//foilBody.AddForceAtPosition(-Physics.gravity/floaterCount, transform.position, ForceMode.Acceleration);
        float waterLevel = waterPlane.position.y;

		if (transform.position.y < waterLevel) // if boat is underwater (speed low)
        {
            float displacementMultiplyer = Mathf.Clamp01(waterLevel - transform.position.y);
			Vector3 buoyancyForce = new Vector3(0f, displacementMultiplyer * waterDensity * floatingThreshold * Mathf.Abs(Physics.gravity.y), 0f);
			// With pitch and roll frozen (arcade boat), torque on those axes can't tilt
			// the body and PhysX leaks it into yaw instead: the boat spins at the
			// angular velocity cap. So push through the centre of mass and leave the
			// angular damping out; yaw is then TwinMotorController's to set.
			const RigidbodyConstraints tilt = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
			bool tiltFrozen = (foilBody.constraints & tilt) == tilt;
			Vector3 at = tiltFrozen ? foilBody.worldCenterOfMass : transform.position;
			foilBody.AddForceAtPosition(buoyancyForce, at, ForceMode.Acceleration);

			foilBody.AddForce(Vector3.up * -foilBody.linearVelocity.y * waterDrag, ForceMode.VelocityChange);
			if (!tiltFrozen)
				foilBody.AddTorque(-foilBody.angularVelocity * waterAngularDrag, ForceMode.VelocityChange);
		}
    }
}
