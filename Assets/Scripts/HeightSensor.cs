using UnityEngine;

public class HeightSensor : MonoBehaviour
{
    public float maxDistance = 25f; // Maximum distance the sensor can measure
    private float distance;

    void Update()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, maxDistance))
        {
            distance = hit.distance;
            Debug.DrawRay(transform.position, transform.forward * hit.distance, Color.green);
        }
        else
        {
            distance = maxDistance; // If nothing is hit, set distance to maxDistance
            Debug.DrawRay(transform.position, transform.forward * maxDistance, Color.blue);
        }
    }

    public float GetDistance()
    {
        return distance;
    }
}
