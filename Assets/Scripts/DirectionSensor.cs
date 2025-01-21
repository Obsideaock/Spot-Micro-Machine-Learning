using UnityEngine;

public class DirectionSensor : MonoBehaviour
{
	private float overlap;
    public Transform targetObject;

    void Update()
    {
        Debug.DrawRay(transform.position, transform.forward * 30, Color.blue);
        Vector3 directionOfTarget = (targetObject.position - transform.position).normalized;
        Debug.DrawRay(transform.position, directionOfTarget * 30, Color.blue);
        // Calculate the similarity between the two directions
        overlap = Vector3.Dot(transform.forward.normalized, directionOfTarget);
		overlap = (overlap + 1) / 2 * 100;
    }

    public float GetOverlap()
    {
        return overlap;
    }
}
