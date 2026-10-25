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
            Debug.Log("Underwater");
            float displacementMultiplyer = Mathf.Clamp01(waterLevel - transform.position.y);
			Vector3 buoyancyForce = new Vector3(0f, displacementMultiplyer * waterDensity * floatingThreshold * Mathf.Abs(Physics.gravity.y), 0f);
			Debug.Log(buoyancyForce);
			foilBody.AddForceAtPosition(buoyancyForce, transform.position, ForceMode.Acceleration);

			foilBody.AddForce(Vector3.up * -foilBody.linearVelocity.y * waterDrag, ForceMode.VelocityChange);
			foilBody.AddTorque(-foilBody.angularVelocity * waterAngularDrag, ForceMode.VelocityChange);
		}
    }
}
