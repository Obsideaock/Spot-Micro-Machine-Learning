using UnityEngine;

public class AIHingeJointController : MonoBehaviour
{
    // Use the "new" keyword to hide any inherited member with the same name.
    public new HingeJoint hingeJoint;

    // Motor settings.
    public float maxMotorForce = 10f;
    public float minAngle = -45f;
    public float maxAngle = 45f;

    // The desired target angle.
    private float targetAngle;

    /// <summary>
    /// Returns the current angle of the hinge joint.
    /// </summary>
    public float CurrentAngle
    {
        get { return hingeJoint ? hingeJoint.angle : 0f; }
    }

    private void Awake()
    {
        if (hingeJoint == null)
        {
            hingeJoint = GetComponent<HingeJoint>();
        }
        // Set up joint limits.
        JointLimits limits = hingeJoint.limits;
        limits.min = minAngle;
        limits.max = maxAngle;
        hingeJoint.limits = limits;
        hingeJoint.useLimits = true;
    }

    /// <summary>
    /// Sets the target angle for the joint and updates the motor.
    /// </summary>
    /// <param name="angle">Desired target angle (in degrees).</param>
    public void SetTargetAngle(float angle)
    {
        targetAngle = Mathf.Clamp(angle, minAngle, maxAngle);
        UpdateMotor();
    }

    /// <summary>
    /// Updates the motor parameters based on the current target angle.
    /// </summary>
    private void UpdateMotor()
    {
        float velocity = (targetAngle - CurrentAngle) * 1f; // Adjust multiplier if needed.
        JointMotor motor = hingeJoint.motor;
        motor.force = maxMotorForce;
        motor.targetVelocity = velocity;
        hingeJoint.motor = motor;
        hingeJoint.useMotor = true;
    }
}
