using UnityEngine;

public class Accelerometer : MonoBehaviour
{
    // This will hold our simulated accelerometer reading.
    private Vector3 properAcceleration;

    void Update()
    {
        // In world space, a non-freefalling object “feels” an acceleration equal to -gravity.
        // Convert that vector into the object’s local space.
        properAcceleration = transform.InverseTransformDirection(-Physics.gravity);

        // For a device at rest (and flat), properAcceleration will be ~ (0, 9.81, 0)
        // indicating that the Y axis is the up/down axis.
        // Debug.Log("Proper acceleration: " + properAcceleration.ToString("F3"));
    }

    // These public methods return the individual components.
    public float GetAccX()
    {
        return properAcceleration.x;
    }

    public float GetAccY()
    {
        return properAcceleration.y;
    }

    public float GetAccZ()
    {
        return properAcceleration.z;
    }
}
