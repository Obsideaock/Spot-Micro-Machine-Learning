using UnityEngine;

public class AIHingeJointController : MonoBehaviour
{
    public new HingeJoint hingeJoint; // Use the 'new' keyword to hide inherited member
    public float maxMotorForce = 10.0f;
    public float minAngle = -45.0f;
    public float maxAngle = 45.0f;

    private float targetAngle = 0.0f;

    [SerializeField, ReadOnly]
    private float currentAngle = 0.0f;

    void Start()
    {
        if (hingeJoint == null)
        {
            hingeJoint = GetComponent<HingeJoint>();
        }

        JointLimits limits = hingeJoint.limits;
        limits.min = minAngle;
        limits.max = maxAngle;
        hingeJoint.limits = limits;
        hingeJoint.useLimits = true;
    }

    void Update()
    {
        currentAngle = hingeJoint.angle;
    }

    public void SetTargetAngle(float angle)
    {
        targetAngle = Mathf.Clamp(angle, minAngle, maxAngle);
        UpdateMotor();
    }

    void UpdateMotor()
    {
        JointMotor motor = hingeJoint.motor;
        motor.targetVelocity = CalculateTargetVelocity();
        motor.force = maxMotorForce;
        hingeJoint.motor = motor;
        hingeJoint.useMotor = true;
    }

    float CalculateTargetVelocity()
    {
        float velocity = (targetAngle - currentAngle) * 1f; // Adjust the multiplier as needed
        return velocity;
    }
}
