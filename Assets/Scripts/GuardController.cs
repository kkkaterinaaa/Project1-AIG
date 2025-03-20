using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GuardController : MonoBehaviour
{
    // Guard AI States
    public enum AIState { Patrol, Chase, Return }
    private AIState _currentState = AIState.Patrol;

    #region Patrol Settings
    [Header("Patrol Settings")]
    // Waypoints provided as empty GameObjects.
    public Transform[] waypoints;
    private List<Vector3> _patrolPoints = new List<Vector3>();
    private int _currentPatrolIndex = 0;
    #endregion

    #region A* Pathfinding for Chase Mode
    [Header("A* Pathfinding")]
    public AStarPathfinder pathfinder;
    public float pathUpdateInterval = 1f;  // How often to update the path when chasing
    private List<Vector3> _currentPath = new List<Vector3>();
    private int _currentWaypointIndex = 0;
    public float waypointThreshold = 0.7f;
    public float chaseTimeout = 5f;        // How long to chase if the player is lost
    private float _chaseTimer = 0f;
    private Vector3 _lastKnownPlayerPos;
    private bool _hasLostPlayer = false;
    #endregion

    #region Detection Settings
    [Header("Detection")]
    public string playerTag = "Player";
    public Transform player;              // Can assign in Inspector or found via tag in Start()
    public float detectionRadius = 10f;   // Horizontal detection range
    public float viewAngle = 45f;         // Field-of-view for detection
    #endregion

    #region Movement Settings
    [Header("Movement")]
    public float patrolSpeed = 3.5f;
    public float chaseSpeed = 3f;
    public float rotationSpeed = 5f;      // Used for patrol mode
    public float chaseRotSpeed = 120f;    // Used in chase mode for faster turning
    public float stoppingDistance = 3f;   // When chasing, if within this distance of the player, stop moving
    public float homeArrivalThreshold = 1f; // When returning, if within this distance from patrol point, consider arrived
    private Vector3 _velocity = Vector3.zero;
    #endregion

    #region Animation & Misc
    [Header("Animation")]
    private Animator _animator;
    private bool _isWalking = false;
    #endregion

    void Start()
    {
        // Locate the player if not assigned.
        if (player == null)
        {
            GameObject pObj = GameObject.FindGameObjectWithTag(playerTag);
            if (pObj != null)
                player = pObj.transform;
            else
                Debug.LogWarning("Player with tag '" + playerTag + "' not found.");
        }
        _animator = GetComponent<Animator>();

        // Initialize patrol points from the assigned empty GameObjects.
        InitializePatrolPoints();
        if (_patrolPoints.Count > 0)
        {
            // Start patrol from the first patrol point.
            UpdatePath(_patrolPoints[_currentPatrolIndex]);
        }
        _currentState = AIState.Patrol;
    }

    // Populate _patrolPoints from the positions of the empty waypoint objects.
    void InitializePatrolPoints()
    {
        _patrolPoints.Clear();
        foreach (Transform wp in waypoints)
        {
            _patrolPoints.Add(wp.position);
        }
    }

    void Update()
    {
        if (player == null)
            return;

        // State management: Patrol, Chase, or Return.
        switch (_currentState)
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

        // Apply movement.
        transform.position += _velocity * Time.deltaTime;
        SnapToGround();
    }

    #region Patrol Behavior
    void PatrolUpdate()
    {
        // Follow the current patrol path.
        FollowPath(patrolSpeed, rotationSpeed);
        // When the path is complete, move to the next patrol point.
        if (_currentPath == null || _currentWaypointIndex >= _currentPath.Count)
        {
            _currentPatrolIndex = (_currentPatrolIndex + 1) % _patrolPoints.Count;
            UpdatePath(_patrolPoints[_currentPatrolIndex]);
        }
    }

    // Check if the player is within detection range and view angle.
    void LookForPlayer()
    {
        Vector3 flatGuardPos = new Vector3(transform.position.x, 0, transform.position.z);
        Vector3 flatPlayerPos = new Vector3(player.position.x, 0, player.position.z);
        float distance = Vector3.Distance(flatGuardPos, flatPlayerPos);
        if (distance < detectionRadius)
        {
            Vector3 dirToPlayer = player.position - transform.position;
            if (Vector3.Angle(transform.forward, dirToPlayer) < viewAngle)
            {
                StartChase();
            }
        }
    }
    #endregion

    #region Chase Behavior
    void StartChase()
    {
        _currentState = AIState.Chase;
        _hasLostPlayer = false;
        _chaseTimer = chaseTimeout;
        UpdatePath(player.position);
        StartCoroutine(UpdatePathRoutine());
    }

    IEnumerator UpdatePathRoutine()
    {
        // Update A* path periodically while chasing.
        while (_currentState == AIState.Chase)
        {
            if (player != null)
            {
                if (!_hasLostPlayer || Vector3.Distance(player.position, _lastKnownPlayerPos) > 1f)
                {
                    UpdatePath(player.position);
                    _lastKnownPlayerPos = player.position;
                }
            }
            yield return new WaitForSeconds(pathUpdateInterval);
        }
    }

    void ChaseUpdate()
    {
        // Determine horizontal distance to player.
        Vector3 flatGuard = new Vector3(transform.position.x, 0, transform.position.z);
        Vector3 flatPlayer = new Vector3(player.position.x, 0, player.position.z);
        float distToPlayer = Vector3.Distance(flatGuard, flatPlayer);
        if (distToPlayer <= detectionRadius)
        {
            UpdatePath(player.position);
            _hasLostPlayer = false;
            _chaseTimer = chaseTimeout;
        }
        else
        {
            if (!_hasLostPlayer)
            {
                _hasLostPlayer = true;
                _chaseTimer = chaseTimeout;
                _lastKnownPlayerPos = player.position;
            }
            else
            {
                _chaseTimer -= Time.deltaTime;
                if (_chaseTimer <= 0f)
                {
                    StartReturn();
                    return;
                }
            }
        }

        // If too close to player, stop moving.
        if (distToPlayer <= stoppingDistance)
        {
            _velocity = Vector3.zero;
            _animator.SetBool("Walk_Anim", false);
            return;
        }
        FollowPath(chaseSpeed, chaseRotSpeed);
    }
    #endregion

    #region Return Behavior
    void StartReturn()
    {
        _currentState = AIState.Return;
        int nearestIndex = GetNearestPatrolPointIndex();
        _currentPatrolIndex = nearestIndex;
        UpdatePath(_patrolPoints[nearestIndex]);
    }

    int GetNearestPatrolPointIndex()
    {
        int nearestIndex = 0;
        float minDistance = Vector3.Distance(transform.position, _patrolPoints[0]);
        for (int i = 1; i < _patrolPoints.Count; i++)
        {
            float distance = Vector3.Distance(transform.position, _patrolPoints[i]);
            if (distance < minDistance)
            {
                minDistance = distance;
                nearestIndex = i;
            }
        }
        return nearestIndex;
    }

    void ReturnUpdate()
    {
        FollowPath(patrolSpeed, rotationSpeed);
        if (_currentPath == null || _currentWaypointIndex >= _currentPath.Count)
        {
            ResumePatrol();
        }
    }

    void ResumePatrol()
    {
        _currentState = AIState.Patrol;
        if (_patrolPoints.Count > 0)
        {
            UpdatePath(_patrolPoints[_currentPatrolIndex]);
        }
    }
    #endregion

    #region Path & Movement Helpers
    // Update the A* path from current position to the target.
    void UpdatePath(Vector3 target)
    {
        _currentPath = pathfinder.FindPath(transform.position, target);
        _currentWaypointIndex = 0;
    }

    // Follows the current path using a given speed and rotation rate.
    void FollowPath(float moveSpeed, float rotSpeedParam)
    {
        if (_currentPath == null || _currentPath.Count == 0 || _currentWaypointIndex >= _currentPath.Count)
        {
            UpdateAnimation(false);
            return;
        }
        Vector3 waypoint = _currentPath[_currentWaypointIndex];
        // Lock Y value to ensure horizontal movement.
        waypoint.y = transform.position.y;
        float distanceToWaypoint = Vector3.Distance(transform.position, waypoint);
        if (distanceToWaypoint < waypointThreshold)
        {
            _currentWaypointIndex++;
            if (_currentWaypointIndex >= _currentPath.Count)
            {
                UpdateAnimation(false);
                return;
            }
            waypoint = _currentPath[_currentWaypointIndex];
            waypoint.y = transform.position.y;
        }
        MoveTowardsTarget(waypoint, moveSpeed, rotSpeedParam);
    }

    // Moves the guard toward the target while smoothly rotating.
    void MoveTowardsTarget(Vector3 target, float moveSpeed, float rotSpeedParam)
    {
        Vector3 desiredDirection = (target - transform.position).normalized;
        // Smooth out direction changes.
        Vector3 smoothDirection = Vector3.Lerp(transform.forward, desiredDirection, 0.1f);
        Quaternion targetRotation = Quaternion.LookRotation(smoothDirection, Vector3.up);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotSpeedParam * Time.deltaTime);
        float angleDifference = Vector3.Angle(transform.forward, desiredDirection);
        if (angleDifference < 2f)
        {
            transform.position += transform.forward * moveSpeed * Time.deltaTime;
            UpdateAnimation(true);
        }
        else
        {
            UpdateAnimation(false);
        }
    }
    #endregion

    #region Animation & Debugging
    void UpdateAnimation(bool walking)
    {
        if (_isWalking != walking)
        {
            _isWalking = walking;
            _animator.SetBool("Walk_Anim", _isWalking);
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        if (_currentPath != null)
        {
            for (int i = 0; i < _currentPath.Count - 1; i++)
            {
                Gizmos.DrawLine(_currentPath[i], _currentPath[i + 1]);
            }
        }
        if (player != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position, detectionRadius);
        }
    }
    #endregion

    // Keeps the guard grounded by raycasting downward.
    void SnapToGround()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position + Vector3.up * 2f, Vector3.down, out hit, 10f))
        {
            transform.position = new Vector3(transform.position.x, hit.point.y, transform.position.z);
        }
    }
}
