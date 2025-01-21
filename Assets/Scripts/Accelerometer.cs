using UnityEngine;
using System.Collections;

public class Accelerometer : MonoBehaviour
{
    // private Vector3 previousVelocity;
    // private Vector3 currentVelocity;
    // private Vector3 previousPosition;
    // private Vector3 acceleration;
    private float updateInterval = 0.0f; // Update interval in seconds

    // Variables to store rotation
    private float rotationX;
    private float rotationY;
    private float rotationZ;

    // Start is called before the first frame update
    void Start()
    {
        // previousVelocity = Vector3.zero;
        // currentVelocity = Vector3.zero;
        // previousPosition = transform.position;
        // acceleration = Vector3.zero;
        StartCoroutine(UpdateRotation());
    }

    IEnumerator UpdateRotation()
    {
        while (true)
        {
            yield return new WaitForSeconds(updateInterval);

            // Calculate the current rotation of the game object
            rotationY = (transform.rotation.eulerAngles.y + 180) % 360;
            rotationX = transform.rotation.eulerAngles.x;
            rotationZ = transform.rotation.eulerAngles.z;

            // Uncomment if you want to see the rotations in the console
            // Debug.Log($"Rotation X: {rotationX}, Rotation Y: {rotationY}, Rotation Z: {rotationZ}");

            // Update the previous velocity and position for the next frame
            // previousVelocity = currentVelocity;
            // previousPosition = transform.position;
        }
    }

    // Method to get the rotation in the x direction
    public float GetAccX()
    {
        return rotationX;
    }

    // Method to get the rotation in the y direction
    public float GetAccY()
    {
        return rotationY;
    }

    // Method to get the rotation in the z direction
    public float GetAccZ()
    {
        return rotationZ;
    }
}
