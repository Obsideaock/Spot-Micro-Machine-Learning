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

    // For servo velocity observations.
    private float[] previousJointAngles;
    private const float maxServoAngularVelocity = 360f;

    public bool isAlive { get; private set; }

    public override void Initialize()
    {
        isAlive = true;
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
        //isAlive = true;
        ResetAgent();
        ResetTarget();
        previousDistanceToTarget = Vector3.Distance(mainBody.position, targetObject.position);
        targetRewardMultiplier = 1.0f;
        StartCoroutine(ResetFloorMaterialWithDelay());
    }

    public void ReactivateAgent()
    {
        isAlive = true;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // --- Group 1: Joint Observations (angles and angular velocities) ---
        string jointLog = "Joint Observations: ";
        for (int i = 0; i < hingeJointControllers.Length; i++)
        {
            var controller = hingeJointControllers[i];
            float currentAngle = controller.hingeJoint.angle;
            float normalizedAngle = ((currentAngle - controller.minAngle) / (controller.maxAngle - controller.minAngle)) * 2f - 1f;
            sensor.AddObservation(normalizedAngle);

            float angularVelocity = (currentAngle - previousJointAngles[i]) / Time.deltaTime;
            previousJointAngles[i] = currentAngle;
            float normalizedAngularVelocity = angularVelocity / maxServoAngularVelocity;
            sensor.AddObservation(normalizedAngularVelocity);

            jointLog += $"[Joint {i}: Angle={normalizedAngle:F2}, AngularVel={normalizedAngularVelocity:F2}] ";
        }
        //Debug.Log(jointLog);

        // --- Group 2: Accelerometer & Height Sensor ---
        float accX = accelerometer.GetAccX();
        float accY = accelerometer.GetAccY();
        float accZ = accelerometer.GetAccZ();
        float normalizedAccX = (accX) / 10f;
        float normalizedAccY = (accY) / 10f;
        float normalizedAccZ = (accZ) / 10f;
        sensor.AddObservation(normalizedAccX);
        sensor.AddObservation(normalizedAccY);
        sensor.AddObservation(normalizedAccZ);

        float normalizedHeight = heightSensor.GetDistance() / 25f;
        sensor.AddObservation(normalizedHeight);

        //Debug.Log($"Accelerometer: X={normalizedAccX:F2}, Y={normalizedAccY:F2}, Z={normalizedAccZ:F2}; Height: {normalizedHeight:F2}");

        // --- Group 3: Distance & Ultrasonic Sensors ---
        Vector3 directionToTarget = targetObject.position - mainBody.position;
        float distanceToTarget = directionToTarget.magnitude;
        if (distanceToTarget > 400f)
        {
            distanceToTarget = 400f;
        }
        float normalizedDistance = distanceToTarget / 160f;
        if (normalizedDistance < 1f)
        {
            normalizedDistance = 1f;
        }
        sensor.AddObservation(normalizedDistance);

        float normalizedUltrasonicLeft = ultrasonicSensorLeft.GetDistance() / 40f;
        sensor.AddObservation(normalizedUltrasonicLeft);
        float normalizedUltrasonicRight = ultrasonicSensorRight.GetDistance() / 40f;
        sensor.AddObservation(normalizedUltrasonicRight);

        //Debug.Log($"Distance to Target: {normalizedDistance:F2}, Ultrasonic Left: {normalizedUltrasonicLeft:F2}, Ultrasonic Right: {normalizedUltrasonicRight:F2}");

        // --- Group 4: Overlap & Main Body Angular Velocity ---
        float normalizedOverlap = (directionSensor.GetOverlap() / 50f) - 1f;
        sensor.AddObservation(normalizedOverlap);

        Rigidbody mainBodyRb = mainBody.GetComponent<Rigidbody>();
        float angularVelocityX = mainBodyRb.angularVelocity.x / 10f;
        float angularVelocityY = mainBodyRb.angularVelocity.y / 10f;
        float angularVelocityZ = mainBodyRb.angularVelocity.z / 10f;
        sensor.AddObservation(angularVelocityX);
        sensor.AddObservation(angularVelocityY);
        sensor.AddObservation(angularVelocityZ);

        //Debug.Log($"Overlap: {normalizedOverlap:F2}, Main Body Angular Velocity: X={angularVelocityX:F2}, Y={angularVelocityY:F2}, Z={angularVelocityZ:F2}");

        // --- Group 5: Main Body Linear Velocity ---
        // Get the local velocity relative to mainBody's rotation
        Vector3 localVelocity = mainBodyRb.transform.InverseTransformDirection(mainBodyRb.velocity);

        // Normalize each component (assuming the intended range is -10 to 10 mapped to -1 to 1)
        // Note: Dividing by -10f inverts the sign; if that’s intentional, keep it.
        float velocityX = localVelocity.x / -10f;
        float velocityY = localVelocity.y / -10f;
        float velocityZ = localVelocity.z / -10f;

        sensor.AddObservation(velocityX);
        sensor.AddObservation(velocityY);
        sensor.AddObservation(velocityZ);


        //Debug.Log($"Main Body Linear Velocity: X={velocityX:F2}, Y={velocityY:F2}, Z={velocityZ:F2}");
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var continuousActions = actions.ContinuousActions;
        for (int i = 0; i < hingeJointControllers.Length; i++)
        {
            float mappedAngle = Mathf.Lerp(hingeJointControllers[i].minAngle,
                                             hingeJointControllers[i].maxAngle,
                                             (continuousActions[i] + 1f) / 2f);
            hingeJointControllers[i].SetTargetAngle(mappedAngle);
        }

        CalculateRewards();
        timeSinceTargetMoved += Time.deltaTime;
    }

    private void CalculateRewards()
    {
        Rigidbody mainBodyRb = mainBody.GetComponent<Rigidbody>();

        float uprightedness = (accelerometer.GetAccY()) / 10f;

        float overlap = (directionSensor.GetOverlap() / 50f) - 1f;

        float height = heightSensor.GetDistance() / 25f;
        
        if (height > .25f)
        {
            height = 1f;
        }else if (height <= .25f)
        {
            height = height / .25f;
        }

        Vector3 directionToTarget = (targetObject.position - mainBody.position).normalized;
        Vector3 velocity = mainBodyRb.velocity;
        float velocityTowardTarget = Vector3.Dot(velocity, directionToTarget);
        // negitive stuff is making weird.
        float rewards = velocityTowardTarget * uprightedness * overlap;
        AddReward(rewards);
    }

    public void OnBodyCollisionWithTarget()
    {
        // Calculate elapsed whole seconds since the target moved.
        int secondsElapsed = Mathf.FloorToInt(timeSinceTargetMoved);

        // Start with a base reward of 50 and subtract 1 point per elapsed second.
        float reward = 50f - secondsElapsed;

        // Ensure the reward doesn't go below 20 points.
        reward = Mathf.Max(reward, 20f);

        AddReward(reward*targetRewardMultiplier);
        floorMeshRenderer.material = winMaterial;

        // Move the target and reset the timer.
        MoveTarget();
        StartCoroutine(ResetFloorMaterialWithDelay());
    }

    public void OnBodyCollisionWithWall()
    {
        isAlive = false;
        AddReward(-50.0f);
        floorMeshRenderer.material = loseMaterial;
        EndEpisode();
        StartCoroutine(ResetFloorMaterialWithDelay());
        StartCoroutine(Died());
    }

    private void ResetAgent()
    {
        //isAlive = true;
        Renderer planeRenderer = plane.GetComponent<Renderer>();
        Bounds planeBounds = planeRenderer.bounds;
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
        float distanceToTarget = Random.Range(10f, 20f);
        float angleOffset = Random.Range(-10f, 10f);
        Quaternion randomRotation = Quaternion.Euler(0f, angleOffset + 180f, 0f);
        Vector3 forwardDirection = (randomRotation * spotMicro.forward).normalized;
        Vector3 newTargetPosition = spotMicro.position + forwardDirection * distanceToTarget;
        newTargetPosition.y = targetObject.position.y;
        targetObject.position = newTargetPosition;
        previousDistanceToTarget = Vector3.Distance(spotMicro.position, targetObject.position);
        timeSinceTargetMoved = 0f;
    }

    private void MoveTarget()
    {
        // Determine a random distance and a slight random angle offset.
        float distanceToTarget = Random.Range(10f, 20f);
        float angleOffset = Random.Range(-10f, 10f);

        // Compute the intended new target position.
        Quaternion rotationAdjustment = Quaternion.Euler(0, angleOffset, 0);
        Vector3 directionToTarget = (targetObject.position - mainBody.position).normalized;
        Vector3 computedTargetPosition = targetObject.position + (rotationAdjustment * directionToTarget) * distanceToTarget;

        // Get the bounds of the plane.
        Renderer planeRenderer = plane.GetComponent<Renderer>();
        Bounds planeBounds = planeRenderer.bounds;

        // Check if the computed target is outside the plane bounds.
        bool offPlane = computedTargetPosition.x < planeBounds.min.x ||
                        computedTargetPosition.x > planeBounds.max.x ||
                        computedTargetPosition.z < planeBounds.min.z ||
                        computedTargetPosition.z > planeBounds.max.z;

        Vector3 newTargetPosition = computedTargetPosition;

        // If it is off the plane, mirror its displacement relative to the agent.
        if (offPlane)
        {
            Vector3 offset = computedTargetPosition - mainBody.position;
            newTargetPosition = mainBody.position - offset; // Reflects the computed offset.
        }

        // Ensure the target's Y position remains unchanged.
        newTargetPosition.y = targetObject.position.y;

        // Update the target's position.
        targetObject.position = newTargetPosition;

        // Reset the tracking variables.
        previousDistanceToTarget = Vector3.Distance(mainBody.position, targetObject.position);
        timeSinceTargetMoved = 0f;
    }

    private IEnumerator ResetFloorMaterialWithDelay()
    {
        yield return new WaitForSeconds(2f);
        floorMeshRenderer.material = normalMaterial;
    }

    private IEnumerator Died()
    {
        yield return new WaitForSeconds(2f);
        isAlive = true;
    }
}
