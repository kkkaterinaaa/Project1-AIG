using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WaypointNavigation: MonoBehaviour
{
    public enum AIState { Patrol, Chase, Return }

    [Header("A* Pathfinding")]
    public AStarPathfinder pathfinder;
    public GridManager gridManager;
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

    [Header("Grid-based Waypoints")]
    public Vector2Int[] gridWaypoints;
    private List<Vector3> patrolPoints = new List<Vector3>();
    private int currentPatrolIndex = 0;

    [Header("Animation")]
    private Animator animator;
    private bool isWalking = false;

    private AIState currentState = AIState.Patrol;
    private float chaseTimer = 0f;
    private Vector3 lastKnownPosition;
    private bool hasLostPlayer = false;

    void Start()
    {
        animator = GetComponent<Animator>();
        InitializePatrolPoints();

        if (patrolPoints.Count > 0)
        {
            UpdatePath(patrolPoints[currentPatrolIndex]);
        }
    }

    void InitializePatrolPoints()
    {
        foreach (Vector2Int wp in gridWaypoints)
        {
            Vector3 worldPos = gridManager.GetWorldPosition(wp.x, wp.y);
            patrolPoints.Add(worldPos);
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

        if (currentPath.Count == 0 || currentWaypointIndex >= currentPath.Count)
        {
            GetNextPatrolPoint();
            UpdatePath(patrolPoints[currentPatrolIndex]);
        }
    }

    private void GetNextPatrolPoint()
    {
        currentPatrolIndex = (currentPatrolIndex + 1) % patrolPoints.Count;
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
        if (patrolPoints.Count > 0)
        {
            int nearestIndex = GetNearestPatrolPointIndex();
            currentPatrolIndex = nearestIndex;
            UpdatePath(patrolPoints[nearestIndex]);
        }
    }

    private int GetNearestPatrolPointIndex()
    {
        int nearestIndex = 0;
        float minDistance = Vector3.Distance(transform.position, patrolPoints[0]);

        for (int i = 1; i < patrolPoints.Count; i++)
        {
            float distance = Vector3.Distance(transform.position, patrolPoints[i]);
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
        if (patrolPoints.Count > 0)
        {
            UpdatePath(patrolPoints[currentPatrolIndex]);
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
            UpdateAnimation(false);
            return;
        }

        Vector3 waypoint = currentPath[currentWaypointIndex];
        waypoint.y = transform.position.y;
        float distanceToWaypoint = Vector3.Distance(transform.position, waypoint);

        if (distanceToWaypoint < waypointThreshold)
        {
            currentWaypointIndex++;
            if (currentWaypointIndex >= currentPath.Count)
            {
                UpdateAnimation(false);
                return;
            }
        }

        MoveTowardsTarget(waypoint);
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

        // Move only if facing the target direction correctly
        float angleDifference = Vector3.Angle(transform.forward, direction);
        if (angleDifference < 2f) // Only move when almost aligned with the target
        {
            transform.position += transform.forward * movementSpeed * Time.deltaTime;
            UpdateAnimation(true);
        }
        else
        {
            UpdateAnimation(false); // Idle while rotating
        }
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
