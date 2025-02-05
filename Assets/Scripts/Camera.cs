using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AgentFollowCamera : MonoBehaviour
{
    [Header("Agent Follow Settings")]
    public SpotMicroAgent[] agentsToFollow;

    [Header("Orbit Settings")]
    public float distance = 5.0f;
    public float xSpeed = 120.0f;
    public float ySpeed = 120.0f;
    public float yMinLimit = -20f;
    public float yMaxLimit = 80f;
    public float heightOffset = 1.5f;

    [Header("Smoothing Settings")]
    public float smoothTime = 0.2f;
    public float rotationSmoothTime = 0.1f;

    // Internal state for orbiting
    private float x = 0.0f;
    private float y = 0.0f;
    private Vector3 positionSmoothVelocity = Vector3.zero;
    private Quaternion currentRotation;

    // The current agent being followed
    private SpotMicroAgent currentAgent;

    void Start()
    {
        // Choose a random agent from the list if available.
        if (agentsToFollow != null && agentsToFollow.Length > 0)
        {
            int randomIndex = Random.Range(0, agentsToFollow.Length);
            currentAgent = agentsToFollow[randomIndex];
        }
        else
        {
            Debug.LogError("AgentFollowCamera: No agents assigned to follow!");
            enabled = false;
            return;
        }

        // Initialize orbit angles based on the camera’s starting rotation.
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;
        currentRotation = transform.rotation;
    }

    void LateUpdate()
    {
        // If the current agent is not alive (i.e. it “died”),
        // select a new one using your original criteria.
        if (currentAgent == null || !currentAgent.isAlive)
        {
            SpotMicroAgent newAgent = SelectNewAgent();
            if (newAgent != null && newAgent != currentAgent)
            {
                // Before switching, start a coroutine to re-enable the old agent
                // after the camera moves sufficiently away.
                SpotMicroAgent oldAgent = currentAgent;
                currentAgent = newAgent;
                StartCoroutine(ReactivateAgentAfterCameraMovesAway(oldAgent));
            }
            else
            {
                // No change – if no candidate is found, do nothing.
                return;
            }
        }

        // The camera follows the main body of the selected agent.
        Transform targetTransform = currentAgent.mainBody;

        // Update orbit angles based on mouse input when the left mouse button is held.
        if (Input.GetMouseButton(0))
        {
            x += Input.GetAxis("Mouse X") * xSpeed * Time.deltaTime;
            y -= Input.GetAxis("Mouse Y") * ySpeed * Time.deltaTime;
            y = ClampAngle(y, yMinLimit, yMaxLimit);
        }

        // Define the orbit center using the target's position and a vertical offset.
        Vector3 orbitCenter = targetTransform.position + Vector3.up * heightOffset;

        // Calculate the desired orbit rotation from the angles.
        Quaternion desiredOrbitRotation = Quaternion.Euler(y, x, 0);

        // Determine the desired camera position relative to the orbit center.
        Vector3 desiredPosition = orbitCenter - (desiredOrbitRotation * Vector3.forward * distance);

        // Smoothly move the camera to the desired position.
        transform.position = Vector3.SmoothDamp(transform.position, desiredPosition, ref positionSmoothVelocity, smoothTime);

        // Instead of facing the orbit center, always look at the target’s main body.
        Quaternion targetLookRotation = Quaternion.LookRotation(targetTransform.position - transform.position);

        // Smoothly interpolate the camera's rotation towards this look rotation.
        currentRotation = Quaternion.Lerp(transform.rotation, targetLookRotation, Time.deltaTime / rotationSmoothTime);
        transform.rotation = currentRotation;
    }

    /// <summary>
    /// Searches for a new agent that meets your criteria.
    /// (For example, you can choose one with the highest cumulative reward.)
    /// </summary>
    private SpotMicroAgent SelectNewAgent()
    {
        List<SpotMicroAgent> livingAgents = new List<SpotMicroAgent>();
        foreach (SpotMicroAgent agent in agentsToFollow)
        {
            // Use whatever criteria you originally use.
            if (agent != null && agent.isAlive)
            {
                livingAgents.Add(agent);
            }
        }

        if (livingAgents.Count == 0)
        {
            Debug.LogWarning("AgentFollowCamera: No living agents found!");
            return null;
        }

        // Example: choose the agent with the highest cumulative reward.
        float highestReward = float.NegativeInfinity;
        SpotMicroAgent bestAgent = null;
        foreach (SpotMicroAgent agent in livingAgents)
        {
            float reward = agent.GetCumulativeReward();
            if (reward > highestReward)
            {
                highestReward = reward;
                bestAgent = agent;
            }
        }
        return bestAgent;
    }

    /// <summary>
    /// Coroutine that waits until the camera has moved far enough from the given agent,
    /// then calls its ReactivateAgent() method to set isAlive back to true.
    /// </summary>
    private IEnumerator ReactivateAgentAfterCameraMovesAway(SpotMicroAgent agent)
    {
        // Define the minimum distance that the camera must be from the agent
        // before we re-enable it for camera selection.
        float requiredDistance = 10f; // adjust this value as needed

        // Wait until the camera is far enough.
        while (Vector3.Distance(transform.position, agent.mainBody.position) < requiredDistance)
        {
            yield return null;
        }

        // Now that the camera is far enough, re-enable the agent.
        agent.ReactivateAgent();
    }

    /// <summary>
    /// Clamps an angle between a minimum and maximum value.
    /// </summary>
    private static float ClampAngle(float angle, float min, float max)
    {
        if (angle < -360f) angle += 360f;
        if (angle > 360f) angle -= 360f;
        return Mathf.Clamp(angle, min, max);
    }
}
