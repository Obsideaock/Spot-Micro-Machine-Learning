using UnityEngine;

public class BodyColliderHandler : MonoBehaviour
{
    public SpotMicroAgent agent;

    private void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<TargetObject>(out TargetObject targetObject))
        {
            agent.OnBodyCollisionWithTarget();
        }
        else if (other.TryGetComponent<Wall>(out Wall wall))
        {
            agent.OnBodyCollisionWithWall();
        }
    }
}
