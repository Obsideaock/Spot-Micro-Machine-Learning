using System.Collections;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class SpotMicroAgent : Agent
{
    public AIHingeJointController[] hingeJointControllers;
    public Transform mainBody;
    public Transform targetObject;
    public Transform plane;
    public Transform spotMicro;

    public Transform body;

    public UltrasonicSensor ultrasonicSensorLeft;
    public UltrasonicSensor ultrasonicSensorRight;
    public DirectionSensor directionSensor;
    public Accelerometer accelerometer;
    public HeightSensor heightSensor;

    public Material winMaterial;
    public Material loseMaterial;
    public Material normalMaterial;
    public MeshRenderer floorMeshRenderer;

    private Vector3 initialMainBodyPosition;
    private Quaternion initialMainBodyRotation;
    private Vector3[] initialPositions;
    private Quaternion[] initialRotations;
    private Rigidbody[] rigidbodies;

    private float timeSinceTargetMoved;
    private float previousDistanceToTarget;
    private float targetRewardMultiplier = 1.0f;

    private float[] previousJointAngles;
    private const float maxServoAngularVelocity = 360f;

    public override void Initialize()
    {
        initialMainBodyPosition = mainBody.position;
        initialMainBodyRotation = mainBody.rotation;

        Transform[] allParts = spotMicro.GetComponentsInChildren<Transform>();
        initialPositions = new Vector3[allParts.Length];
        initialRotations = new Quaternion[allParts.Length];
        rigidbodies = new Rigidbody[allParts.Length];

        for (int i = 0; i < allParts.Length; i++)
        {
            initialPositions[i] = allParts[i].position;
            initialRotations[i] = allParts[i].rotation;
            rigidbodies[i] = allParts[i].GetComponent<Rigidbody>();
        }

        if (body != null)
        {
            var colliderHandler = body.gameObject.AddComponent<BodyColliderHandler>();
            colliderHandler.agent = this;
        }

        timeSinceTargetMoved = 0f;

        previousJointAngles = new float[hingeJointControllers.Length];
        for (int i = 0; i < hingeJointControllers.Length; i++)
        {
            previousJointAngles[i] = hingeJointControllers[i].hingeJoint.angle;
        }
    }

    public override void OnEpisodeBegin()
    {
        ResetAgent();
        ResetTarget();
        previousDistanceToTarget = Vector3.Distance(mainBody.position, targetObject.position);

        // Reset the reward multiplier at the beginning of each episode
        targetRewardMultiplier = 1.0f;

        // Reset floor material after a short delay
        StartCoroutine(ResetFloorMaterialWithDelay());
    }

    public override void CollectObservations(VectorSensor sensor)


    {
        for (int i = 0; i < hingeJointControllers.Length; i++)
        {
            var controller = hingeJointControllers[i];
            float currentAngle = controller.hingeJoint.angle;
            // Normalize the joint angle between -1 and 1.
            float normalizedAngle = ((currentAngle - controller.minAngle) / (controller.maxAngle - controller.minAngle)) * 2f - 1f;
            sensor.AddObservation(normalizedAngle);

            // Calculate angular velocity (change in angle over time).
            float angularVelocity = (currentAngle - previousJointAngles[i]) / Time.deltaTime;
            // Update the previous angle for the next frame.
            previousJointAngles[i] = currentAngle;
            // Normalize the angular velocity observation.
            sensor.AddObservation(angularVelocity / maxServoAngularVelocity);
        }

        float accX = accelerometer.GetAccX();
        float accY = accelerometer.GetAccY();
        float accZ = accelerometer.GetAccZ();

        sensor.AddObservation((accX) / 180f - 1f);
        sensor.AddObservation((accY) / 180f - 1f);
        sensor.AddObservation((accZ) / 180f - 1f);

        sensor.AddObservation(heightSensor.GetDistance() / 20f);

        Vector3 directionToTarget = targetObject.position - mainBody.position;
        float distanceToTarget = directionToTarget.magnitude;

        sensor.AddObservation(distanceToTarget / 40);

        sensor.AddObservation(ultrasonicSensorLeft.GetDistance() / 400);
        sensor.AddObservation(ultrasonicSensorRight.GetDistance() / 400);

        float overlap = (directionSensor.GetOverlap() / 50f) - 1;
        sensor.AddObservation(overlap);

        Rigidbody mainBodyRb = mainBody.GetComponent<Rigidbody>();
        sensor.AddObservation(mainBodyRb.angularVelocity.x / 10f);
        sensor.AddObservation(mainBodyRb.angularVelocity.y / 10f);
        sensor.AddObservation(mainBodyRb.angularVelocity.z / 10f);

        sensor.AddObservation(mainBodyRb.velocity.x / 10f); // X direction velocity
        sensor.AddObservation(mainBodyRb.velocity.y / 10f); // Y direction velocity
        sensor.AddObservation(mainBodyRb.velocity.z / 10f); // Z direction velocity
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var continuousActions = actions.ContinuousActions;
        for (int i = 0; i < hingeJointControllers.Length; i++)
        {
            float targetAngle = Mathf.Lerp(hingeJointControllers[i].minAngle, hingeJointControllers[i].maxAngle, (continuousActions[i] + 1f) / 2f);
            targetAngle = Mathf.Clamp(targetAngle, hingeJointControllers[i].minAngle, hingeJointControllers[i].maxAngle);
            hingeJointControllers[i].SetTargetAngle(targetAngle);
        }

        CalculateRewards();
        timeSinceTargetMoved += Time.deltaTime;
    }

    private void CalculateRewards()
    {
        float uprightedness = (Mathf.Clamp01(Vector3.Dot(mainBody.transform.forward.normalized, Vector3.up)))*2 - 1;
        

        float currentDistanceToTarget = Vector3.Distance(mainBody.position, targetObject.position);
        float distanceDifference = previousDistanceToTarget - currentDistanceToTarget;
       // AddReward(distanceDifference * 5f);
        previousDistanceToTarget = currentDistanceToTarget;

        float overlap = (directionSensor.GetOverlap() / 50f) - 1f;
        //Debug.Log("Overlap" + overlap);
        //AddReward(overlap);

        // Reward for moving in the direction of the target
        Vector3 directionToTarget = (targetObject.position - mainBody.position).normalized;
        Rigidbody mainBodyRb = mainBody.GetComponent<Rigidbody>();
        Vector3 velocity = mainBodyRb.velocity;

        float velocityTowardTarget = Vector3.Dot(velocity, directionToTarget); // Measure velocity in the direction of the target
        //if (velocityTowardTarget < 0)
        //{
        //AddReward(velocityTowardTarget); // Scale reward by the velocity magnitude
        //}
        rewards = velocityTowardTarget * overlap * uprightedness;
        Debug.Log(rewards); 
        AddReward(rewards);
        
    }

    public void OnBodyCollisionWithTarget()
    {
        // Apply the cumulative multiplier to the reward for reaching the target
        AddReward(50 * targetRewardMultiplier);
        floorMeshRenderer.material = winMaterial;

        // Increase the multiplier for the next time the target is reached
        //targetRewardMultiplier += 0.25f;

        MoveTarget();

        // Reset floor material after a short delay
        StartCoroutine(ResetFloorMaterialWithDelay());
    }

    public void OnBodyCollisionWithWall()
    {
        AddReward(-50.0f);
        floorMeshRenderer.material = loseMaterial;
        EndEpisode();

        // Reset floor material after a short delay
        StartCoroutine(ResetFloorMaterialWithDelay());
    }

    private void ResetAgent()
    {
        Renderer planeRenderer = plane.GetComponent<Renderer>();
        Bounds planeBounds = planeRenderer.bounds;

        // Randomize SpotMicro's Y-axis rotation
        

        Vector3 planeCenter = new Vector3(
            (planeBounds.min.x + planeBounds.max.x) / 2f,
            initialMainBodyPosition.y,
            (planeBounds.min.z + planeBounds.max.z) / 2f
        );

        spotMicro.position = planeCenter;

        Transform[] allParts = spotMicro.GetComponentsInChildren<Transform>();
        for (int i = 0; i < allParts.Length; i++)
        {
            allParts[i].position = initialPositions[i] + (planeCenter - initialMainBodyPosition);
            allParts[i].rotation = initialRotations[i];
            if (rigidbodies[i] != null)
            {
                rigidbodies[i].velocity = Vector3.zero;
                rigidbodies[i].angularVelocity = Vector3.zero;
            }
        }

        foreach (var controller in hingeJointControllers)
        {
            controller.hingeJoint.motor = new JointMotor();
            controller.hingeJoint.useMotor = true;
        }

        timeSinceTargetMoved = 0f;

        float randomRotationY = Random.Range(0f, 360f);
        spotMicro.rotation = Quaternion.Euler(0f, randomRotationY, 0f);

        ResetTarget();
    }


    private void ResetTarget()
    {
        // Random distance between 10 and 25 units
        float distanceToTarget = Random.Range(20f, 30f);

        // Randomize the angle offset from the forward direction
        float angleOffset = Random.Range(-10f, 10f); // Adjust range as needed for variety
        Quaternion randomRotation = Quaternion.Euler(0f, angleOffset + 180f, 0f);

        // Compute a new direction with the random angle
        Vector3 forwardDirection = (randomRotation * spotMicro.forward).normalized;

        // Calculate the new target position
        Vector3 newTargetPosition = spotMicro.position + forwardDirection * distanceToTarget;
        newTargetPosition.y = targetObject.position.y;


        // Set the target's position
        targetObject.position = newTargetPosition;

        // Update the previous distance to the target
        previousDistanceToTarget = Vector3.Distance(spotMicro.position, targetObject.position);
        timeSinceTargetMoved = 0f;

    }


    private void MoveTarget()
    {
        float distanceToTarget = Random.Range(25f, 35f);
        float angleOffset = Random.Range(-10f, 10f); // Adjust target direction by ±10 degrees
        Quaternion rotationAdjustment = Quaternion.Euler(0, angleOffset, 0);

        Vector3 directionToTarget = (targetObject.position - mainBody.position).normalized;
        Vector3 adjustedDirection = rotationAdjustment * directionToTarget;
        Vector3 newTargetPosition = targetObject.position + adjustedDirection * distanceToTarget;

        Renderer planeRenderer = plane.GetComponent<Renderer>();
        Bounds planeBounds = planeRenderer.bounds;

        newTargetPosition.x = Mathf.Clamp(newTargetPosition.x, planeBounds.min.x + 1f, planeBounds.max.x - 1f);
        newTargetPosition.z = Mathf.Clamp(newTargetPosition.z, planeBounds.min.z + 1f, planeBounds.max.z - 1f);

        newTargetPosition.y = targetObject.position.y; // Maintain original Y position
        targetObject.position = newTargetPosition;

        previousDistanceToTarget = Vector3.Distance(mainBody.position, targetObject.position);
        timeSinceTargetMoved = 0f;
    }

    private IEnumerator ResetFloorMaterialWithDelay()
    {
        yield return new WaitForSeconds(2f);
        floorMeshRenderer.material = normalMaterial;
    }
}
