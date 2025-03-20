using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WaypointNavigation : MonoBehaviour
{
    public enum AIState { Patrol, Chase, Return }

    [Header("A* Pathfinding")]
    public AStarPathfinder pathfinder;
    public GridManager gridManager; // Used for A* path calculation during chase/return
    private List<Vector3> currentPath = new List<Vector3>();
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

    [Header("Obstacle Avoidance")]
    public float obstacleDetectionDistance = 2f;
    public LayerMask obstacleMask;
    public float avoidanceRotation = 30f;

    [Header("Waypoints (Empty Objects)")]
    public Transform[] waypoints; 

    [Header("Animation")]
    private Animator animator;
    private bool isWalking = false;

    private AIState currentState = AIState.Patrol;
    private float chaseTimer = 0f;
    private Vector3 lastKnownPosition;
    private bool hasLostPlayer = false;
    private Vector3 lastPosition;
    private float stuckTimer = 0f;
    private float stuckThreshold = 2f; // seconds

    void Start()
    {
        animator = GetComponent<Animator>();

        if (waypoints.Length > 0)
        {
            // Start patrolling toward the first waypoint
            UpdatePath(waypoints[currentWaypointIndex].position);
        }
        else
        {
            Debug.LogError("No waypoints assigned.");
        }
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

    private void PatrolUpdate()
    {
        FollowPath();

        // Once the path is complete, move to the next waypoint and update the path
        if (currentPath.Count == 0 || currentWaypointIndex >= currentPath.Count)
        {
            GetNextWaypoint();
            UpdatePath(waypoints[currentWaypointIndex].position);
        }
    }

    private void GetNextWaypoint()
    {
        currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
    }

    private void LookForPlayer()
    {
        Vector3 directionToPlayer = player.position - transform.position;
        float distanceToPlayer = directionToPlayer.magnitude;

        if (distanceToPlayer < detectionRadius)
        {
            float angleToPlayer = Vector3.Angle(transform.forward, directionToPlayer);
            if (angleToPlayer < viewAngle)
            {
                StartChase();
            }
        }
    }

    private void StartChase()
    {
        currentState = AIState.Chase;
        hasLostPlayer = false;
        UpdatePath(player.position);
    }

    private void ChaseUpdate()
    {
        if (Vector3.Distance(transform.position, player.position) <= detectionRadius)
        {
            UpdatePath(player.position);
            hasLostPlayer = false;
        }
        else
        {
            if (!hasLostPlayer)
            {
                hasLostPlayer = true;
                chaseTimer = chaseTimeout;
                lastKnownPosition = player.position;
            }
            else
            {
                chaseTimer -= Time.deltaTime;
                if (chaseTimer <= 0f)
                {
                    StartReturn();
                }
            }
        }

        FollowPath();
    }

    private void StartReturn()
    {
        currentState = AIState.Return;
        if (waypoints.Length > 0)
        {
            int nearestIndex = GetNearestWaypointIndex();
            currentWaypointIndex = nearestIndex;
            UpdatePath(waypoints[nearestIndex].position);
        }
    }

    private int GetNearestWaypointIndex()
    {
        int nearestIndex = 0;
        float minDistance = Vector3.Distance(transform.position, waypoints[0].position);

        for (int i = 1; i < waypoints.Length; i++)
        {
            float distance = Vector3.Distance(transform.position, waypoints[i].position);
            if (distance < minDistance)
            {
                minDistance = distance;
                nearestIndex = i;
            }
        }
        return nearestIndex;
    }

    private void ReturnUpdate()
    {
        FollowPath();

        if (currentPath.Count == 0 || currentWaypointIndex >= currentPath.Count)
        {
            ResumePatrol();
        }
    }

    private void ResumePatrol()
    {
        currentState = AIState.Patrol;
        if (waypoints.Length > 0)
        {
            UpdatePath(waypoints[currentWaypointIndex].position);
        }
    }

    private void UpdatePath(Vector3 target)
    {
        currentPath = pathfinder.FindPath(transform.position, target);
        currentWaypointIndex = 0;
    }

    private void FollowPath()
    {
        if (currentPath == null || currentPath.Count == 0 || currentWaypointIndex >= currentPath.Count)
        {
            //UpdateAnimation(false);
            return;
        }
        UpdateAnimation(true);

        Vector3 waypoint = currentPath[currentWaypointIndex];
        waypoint.y = transform.position.y;
        float distanceToWaypoint = Vector3.Distance(transform.position, waypoint);

        if (distanceToWaypoint < waypointThreshold)
        {
            currentWaypointIndex++;
            if (currentWaypointIndex >= currentPath.Count)
            {
                //UpdateAnimation(false);
                return;
            }
            waypoint = currentPath[currentWaypointIndex];
            waypoint.y = transform.position.y;
        }

        lastPosition = transform.position;

        MoveTowardsWaypoint(waypoint);
    }

    private void MoveTowardsWaypoint(Vector3 target)
    {
        Vector3 direction = (target - transform.position);
        float distance = direction.magnitude;
        direction.Normalize();

        // Arrive behavior: decelerate when close to the waypoint.
        float decelerationDistance = 2f; // Adjust as needed
        float speedFactor = 1f;
        if (distance < decelerationDistance)
        {
            speedFactor = distance / decelerationDistance;
        }

        // Calculate the desired velocity (seek behavior)
        Vector3 desiredVelocity = direction * movementSpeed * speedFactor;
        // Apply simple movement: no physics in this case
        transform.position = Vector3.MoveTowards(transform.position, target, desiredVelocity.magnitude * Time.deltaTime);

        float rotationStep = rotationSpeed * Time.deltaTime;
        Vector3 newDirection = Vector3.RotateTowards(transform.forward, direction, rotationStep, 0.0f);
        transform.rotation = Quaternion.LookRotation(newDirection);
        //UpdateAnimation(desiredVelocity.magnitude > 0.1f);
    }


    private void MoveTowardsTarget(Vector3 target)
    {
        Vector3 direction = (target - transform.position).normalized;

        // Obstacle Detection and Avoidance
        if (CheckObstacleInPath(out Vector3 avoidanceDirection))
        {
            direction = avoidanceDirection.normalized;
        }

        float rotationStep = rotationSpeed * Time.deltaTime;
        Vector3 newDirection = Vector3.RotateTowards(transform.forward, direction, rotationStep, 0.0f);
        transform.rotation = Quaternion.LookRotation(newDirection);


        transform.position += transform.forward * movementSpeed * Time.deltaTime;
        UpdateAnimation(true);

    }

    private bool CheckObstacleInPath(out Vector3 avoidanceDirection)
    {
        Vector3 forward = transform.forward;
        Vector3 left = Quaternion.Euler(0, -avoidanceRotation, 0) * forward;
        Vector3 right = Quaternion.Euler(0, avoidanceRotation, 0) * forward;

        if (Physics.Raycast(transform.position, forward, obstacleDetectionDistance, obstacleMask))
        {
            if (!Physics.Raycast(transform.position, right, obstacleDetectionDistance, obstacleMask))
            {
                avoidanceDirection = right;
                return true;
            }
            else if (!Physics.Raycast(transform.position, left, obstacleDetectionDistance, obstacleMask))
            {
                avoidanceDirection = left;
                return true;
            }
        }

        avoidanceDirection = Vector3.zero;
        return false;
    }

    private void UpdateAnimation(bool walking)
    {
        if (isWalking != walking)
        {
            isWalking = walking;
            animator.SetBool("Walk_Anim", isWalking);
        }
    }

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
