using UnityEngine;

public class GuardSwarm : MonoBehaviour
{
    private enum GuardState { Chase, Return, Idle }
    private GuardState _currentState = GuardState.Idle;

    [Header("Player Settings")]
    public string playerTag = "Player";
    public float fieldOfViewAngle = 360f;
    private Transform _player;

    [Header("Swarm Parameters")]
    public float chaseAttractionStrength = 5f;
    public float repulsionStrength = 2f;
    public float repulsionDistance = 3f;
    public bool avoidOtherGuards = true;

    [Header("Obstacle Avoidance")]
    public LayerMask unwalkableMask;
    public float obstacleRepulsionStrength = 20f;

    [Header("Movement Settings")]
    public float maxSpeed = 3f;
    public float rotSpeed = 180f;
    public float stoppingDistance = 3f;
    public float homeArrivalThreshold = 1f;

    private Vector3 _velocity;
    private Animator _anim;
    private Vector3 _originalPosition;

    void Start()
    {
        GameObject playerObj = GameObject.FindGameObjectWithTag(playerTag);
        if (playerObj != null)
            _player = playerObj.transform;
        else
            Debug.LogWarning("Player with tag '" + playerTag + "' not found.");
        _anim = GetComponent<Animator>();
        _originalPosition = transform.position;
        _currentState = GuardState.Idle;
    }

    void Update()
    {
        if (_player == null)
            return;
        bool playerVisible = IsPlayerVisible();
        if (playerVisible)
        {
            _currentState = GuardState.Chase;
        }
        else
        {
            float distToHome = Vector3.Distance(
                new Vector3(transform.position.x, 0, transform.position.z),
                new Vector3(_originalPosition.x, 0, _originalPosition.z));
            _currentState = (distToHome > homeArrivalThreshold) ? GuardState.Return : GuardState.Idle;
        }
        switch (_currentState)
        {
            case GuardState.Chase:
                ChaseBehavior();
                break;
            case GuardState.Return:
                ReturnBehavior();
                break;
            case GuardState.Idle:
                _velocity = Vector3.zero;
                _anim.SetBool("Walk_Anim", false);
                break;
        }
        transform.position += _velocity * Time.deltaTime;
        SnapToGround();
    }

    bool IsPlayerVisible()
    {
        Vector3 dirToPlayer = (_player.position - transform.position).normalized;
        float angle = Vector3.Angle(transform.forward, dirToPlayer);
        if (angle < fieldOfViewAngle * 0.5f)
        {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, dirToPlayer, out hit))
            {
                return (hit.transform == _player);
            }
        }
        return false;
    }

    void ChaseBehavior()
    {
        Vector3 guardFlat = new Vector3(transform.position.x, 0, transform.position.z);
        Vector3 playerFlat = new Vector3(_player.position.x, 0, _player.position.z);
        float horizontalDistance = Vector3.Distance(guardFlat, playerFlat);
        if (horizontalDistance <= stoppingDistance)
        {
            _velocity = Vector3.zero;
            _anim.SetBool("Walk_Anim", false);
            return;
        }
        Vector3 force = (_player.position - transform.position).normalized * chaseAttractionStrength;
        if (avoidOtherGuards)
        {
            GameObject[] guards = GameObject.FindGameObjectsWithTag("Security");
            foreach (GameObject guard in guards)
            {
                if (guard == this.gameObject)
                    continue;
                Vector3 diff = transform.position - guard.transform.position;
                float dist = diff.magnitude;
                if (dist < repulsionDistance && dist > 0.001f)
                    force += diff.normalized * (repulsionStrength / (dist * dist));
            }
        }
        Collider[] obstacles = Physics.OverlapSphere(transform.position, repulsionDistance, unwalkableMask);
        foreach (Collider obstacle in obstacles)
        {
            Vector3 closestPoint = obstacle.ClosestPoint(transform.position);
            Vector3 diff = transform.position - closestPoint;
            float dist = diff.magnitude;
            if (dist < repulsionDistance && dist > 0.001f)
                force += diff.normalized * (obstacleRepulsionStrength / (dist * dist));
        }
        _velocity += force * Time.deltaTime;
        if (_velocity.magnitude > maxSpeed)
            _velocity = _velocity.normalized * maxSpeed;
        if (_velocity.sqrMagnitude > 0.001f)
        {
            _anim.SetBool("Walk_Anim", true);
            Quaternion targetRotation = Quaternion.LookRotation(_velocity);
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, rotSpeed * Time.deltaTime);
        }
        else
        {
            _anim.SetBool("Walk_Anim", false);
        }
    }

    void ReturnBehavior()
    {
        Vector3 toHome = _originalPosition - transform.position;
        Vector3 force = toHome.normalized * chaseAttractionStrength;
        Collider[] obstacles = Physics.OverlapSphere(transform.position, repulsionDistance, unwalkableMask);
        foreach (Collider obstacle in obstacles)
        {
            Vector3 closestPoint = obstacle.ClosestPoint(transform.position);
            Vector3 diff = transform.position - closestPoint;
            float dist = diff.magnitude;
            if (dist < repulsionDistance && dist > 0.001f)
                force += diff.normalized * (obstacleRepulsionStrength / (dist * dist));
        }
        _velocity += force * Time.deltaTime;
        if (_velocity.magnitude > maxSpeed)
            _velocity = _velocity.normalized * maxSpeed;
        if (_velocity.sqrMagnitude > 0.001f)
        {
            _anim.SetBool("Walk_Anim", true);
            Quaternion targetRotation = Quaternion.LookRotation(_velocity);
            transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, rotSpeed * Time.deltaTime);
        }
        else
        {
            _anim.SetBool("Walk_Anim", false);
        }
    }
    void SnapToGround()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position + Vector3.up * 2f, Vector3.down, out hit, 10f))
        {
            transform.position = new Vector3(transform.position.x, hit.point.y, transform.position.z);
        }
    }
}
