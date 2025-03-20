using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GuardAi : MonoBehaviour
{
    public enum AIState { Patrol, Chase, Return }
    private AIState currentState = AIState.Patrol;

    [Header("A* Pathfinding")]
    public AStarPathfinder pathfinder;
    public GridManager gridManager; // Used by A* (for chase/return)
    private List<Vector3> currentPath = new List<Vector3>();
    private int currentPathIndex = 0;

    [Header("Patrol Waypoints (Empty Objects)")]
    public Transform[] waypoints; // Assign empty GameObjects in order
    private int currentWaypointIndex = 0;

    [Header("Movement Parameters")]
    public float movementSpeed = 3.5f;
    public float rotationSpeed = 5f;
    public float waypointThreshold = 0.5f;

    [Header("Detection Parameters")]
    public Transform player;
    public float detectionRadius = 10f;
    public float viewAngle = 45f;
    public float chaseTimeout = 5f;
    private float chaseTimer = 0f;
    private Vector3 lastKnownPlayerPos;

    [Header("Obstacle Avoidance")]
    public float obstacleDetectionDistance = 2f;
    public LayerMask obstacleMask;
    public float avoidanceRotation = 30f;

    [Header("Animation")]
    private Animator animator;
    private bool isWalking = false;

    // Stuck detection variables (for patrol mode)
    private Vector3 lastPosition;
    private float stuckTimer = 0f;
    private float stuckThreshold = 2f; // seconds

    void Start()
    {
        animator = GetComponent<Animator>();

        if (waypoints.Length == 0)
        {
            Debug.LogError("No patrol waypoints assigned.");
            return;
        }
        // Start in patrol mode: set target to first waypoint.
        UpdatePath(waypoints[currentWaypointIndex].position);
    }

    void Update()
    {
        switch (currentState)
        {
            case AIState.Patrol:
                PatrolUpdate();
                LookForPlayer();
                break;
            case AIState.Chase:
                ChaseUpdate();
                break;
            case AIState.Return:
                ReturnUpdate();
                break;
        }
    }

    #region Patrol

    // Patrol: follow path to current waypoint using a simple seek/arrive behavior.
    private void PatrolUpdate()
    {
        // Follow the current A* path
        FollowPath();

        // When path is finished, update to the next waypoint.
        if (currentPath == null || currentPath.Count == 0 || currentPathIndex >= currentPath.Count)
        {
            GetNextWaypoint();
            UpdatePath(waypoints[currentWaypointIndex].position);
        }
    }

    private void GetNextWaypoint()
    {
        currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
    }

    #endregion

    #region Chase

    // Look for player using detection radius and view angle
    private void LookForPlayer()
    {
        Vector3 toPlayer = player.position - transform.position;
        if (toPlayer.magnitude < detectionRadius)
        {
            float angle = Vector3.Angle(transform.forward, toPlayer);
            if (angle < viewAngle)
            {
                // Player detected: switch to chase state
                StartChase();
            }
        }
    }

    private void StartChase()
    {
        currentState = AIState.Chase;
        chaseTimer = chaseTimeout;
        UpdatePath(player.position);
    }

    // In chase mode, update the path toward the player periodically.
    private void ChaseUpdate()
    {
        // If player is visible, update path continuously.
        if (IsPlayerVisible())
        {
            UpdatePath(player.position);
            chaseTimer = chaseTimeout;
        }
        else
        {
            // Player lost: count down the timer.
            chaseTimer -= Time.deltaTime;
            if (chaseTimer <= 0f)
            {
                StartReturn();
                return;
            }
        }
        FollowPath();
    }

    private bool IsPlayerVisible()
    {
        Vector3 toPlayer = (player.position - transform.position).normalized;
        float angle = Vector3.Angle(transform.forward, toPlayer);
        if (angle < viewAngle * 0.5f)
        {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, toPlayer, out hit, detectionRadius))
            {
                return (hit.transform == player);
            }
        }
        return false;
    }

    #endregion

    #region Return

    private void StartReturn()
    {
        currentState = AIState.Return;
        int nearestIndex = GetNearestWaypointIndex();
        currentWaypointIndex = nearestIndex;
        UpdatePath(waypoints[nearestIndex].position);
    }

    private int GetNearestWaypointIndex()
    {
        int nearest = 0;
        float minDist = Vector3.Distance(transform.position, waypoints[0].position);
        for (int i = 1; i < waypoints.Length; i++)
        {
            float d = Vector3.Distance(transform.position, waypoints[i].position);
            if (d < minDist)
            {
                minDist = d;
                nearest = i;
            }
        }
        return nearest;
    }

    private void ReturnUpdate()
    {
        FollowPath();
        // When reached the return waypoint, resume patrol.
        if (currentPath == null || currentPath.Count == 0 || currentPathIndex >= currentPath.Count)
        {
            ResumePatrol();
        }
    }

    private void ResumePatrol()
    {
        currentState = AIState.Patrol;
        UpdatePath(waypoints[currentWaypointIndex].position);
    }

    #endregion

    #region Path & Movement

    // Updates the current path from the guard's position to the target.
    private void UpdatePath(Vector3 target)
    {
        currentPath = pathfinder.FindPath(transform.position, target);
        currentPathIndex = 0;
    }

    // Follows the current path using a combination of seek/arrive behavior and obstacle avoidance.
    private void FollowPath()
    {
        if (currentPath == null || currentPath.Count == 0 || currentPathIndex >= currentPath.Count)
        {
            UpdateAnimation(false);
            return;
        }

        Vector3 waypoint = currentPath[currentPathIndex];
        waypoint.y = transform.position.y;
        float distToWaypoint = Vector3.Distance(transform.position, waypoint);

        // If close enough to the waypoint, move on.
        if (distToWaypoint < waypointThreshold)
        {
            currentPathIndex++;
            if (currentPathIndex >= currentPath.Count)
            {
                UpdateAnimation(false);
                return;
            }
            waypoint = currentPath[currentPathIndex];
            waypoint.y = transform.position.y;
        }

        // --- Stuck detection (for patrol) ---
        // Only use stuck detection in Patrol state.
        if (currentState == AIState.Patrol)
        {
            if (Vector3.Distance(transform.position, lastPosition) < 0.1f)
            {
                stuckTimer += Time.deltaTime;
                if (stuckTimer > stuckThreshold)
                {
                    Debug.Log("Stuck detected. Forcing next waypoint.");
                    GetNextWaypoint();
                    UpdatePath(waypoints[currentWaypointIndex].position);
                    stuckTimer = 0f;
                    lastPosition = transform.position;
                    return;
                }
            }
            else
            {
                stuckTimer = 0f;
            }
            lastPosition = transform.position;
        }
        // --- End stuck detection ---

        // Move toward the current waypoint using Arrive behavior
        MoveTowardsWaypoint(waypoint);
    }

    private void MoveTowardsWaypoint(Vector3 target)
    {
        Vector3 direction = target - transform.position;
        float distance = direction.magnitude;
        direction.Normalize();

        // Arrive: decelerate when close.
        float decelerationDistance = 2f;
        float speedFactor = 1f;
        if (distance < decelerationDistance)
        {
            speedFactor = distance / decelerationDistance;
        }
        Vector3 desiredVelocity = direction * movementSpeed * speedFactor;

        // Apply movement (using MoveTowards for smooth deceleration)
        transform.position = Vector3.MoveTowards(transform.position, target, desiredVelocity.magnitude * Time.deltaTime);

        // Rotate smoothly toward the target
        float step = rotationSpeed * Time.deltaTime;
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, step);

        UpdateAnimation(desiredVelocity.magnitude > 0.1f);
    }

    #endregion

    #region Animation

    private void UpdateAnimation(bool walking)
    {
        if (isWalking != walking)
        {
            isWalking = walking;
            animator.SetBool("Walk_Anim", isWalking);
        }
    }

    #endregion

    // Debug drawing
    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        if (currentPath != null)
        {
            for (int i = 0; i < currentPath.Count - 1; i++)
            {
                Gizmos.DrawLine(currentPath[i], currentPath[i + 1]);
            }
        }
        if (player != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position, detectionRadius);
        }
    }
}
