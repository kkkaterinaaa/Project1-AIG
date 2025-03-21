using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GuardChase : MonoBehaviour
{
    [Header("A* Pathfinding")]
    public AStarPathfinder pathfinder;
    public float pathUpdateInterval = 1f;

    [Header("Player Detection")]
    public string playerTag = "Player";
    public float fieldOfViewAngle = 270f;

    [Header("Movement")]
    public float speed = 3f;
    public float rotSpeed = 120f;
    public float stoppingDistance = 3f;
    
    [Header("Animation & Delay")]
    public float waitBeforeChase = 3.5f;
    private Animator anim;
    private Transform player;
    private bool canChase = false;

    private List<Vector3> currentPath = null;
    private int currentWaypointIndex = 0;
    private Vector3 lastPlayerPosition = Vector3.zero;
    private float waypointArrivalThreshold = 0.7f;

    public void Start()
    {
        GameObject playerObj = GameObject.FindGameObjectWithTag(playerTag);
        if (playerObj != null)
            player = playerObj.transform;
        else
            Debug.LogWarning("No GameObject found with tag '" + playerTag + "'");

        anim = GetComponent<Animator>();
    }

    public void StartChase()
    {
        GameObject playerObj = GameObject.FindGameObjectWithTag(playerTag);
        if (playerObj != null)
            player = playerObj.transform;
        else
            Debug.LogWarning("No GameObject found with tag '" + playerTag + "'");

        anim = GetComponent<Animator>();
        anim.SetBool("Open_Anim", true);
        anim.SetBool("Walk_Anim", false);
        StartCoroutine(WaitAndStartChase());
    }

    public IEnumerator WaitAndStartChase()
    {
        yield return new WaitForSeconds(waitBeforeChase);
        canChase = true;
        StartCoroutine(UpdatePathRoutine());
    }

    private IEnumerator UpdatePathRoutine()
    {
        while (true)
        {
            if (player != null && IsPlayerVisible())
            {
                if (lastPlayerPosition == Vector3.zero || Vector3.Distance(player.position, lastPlayerPosition) > 1f)
                {
                    currentPath = pathfinder.FindPath(transform.position, player.position);
                    currentWaypointIndex = 0;
                    lastPlayerPosition = player.position;
                }
            }
            yield return new WaitForSeconds(pathUpdateInterval);
        }
    }

    void Update()
    {
        if (!canChase || player == null)
            return;

        if (IsPlayerVisible() && currentPath != null && currentPath.Count > 0)
            FollowPath();
        else
            anim.SetBool("Walk_Anim", false);
    }

    public bool IsPlayerVisible()
    {
        Vector3 dirToPlayer = (player.position - transform.position).normalized;
        float angle = Vector3.Angle(transform.forward, dirToPlayer);

        if (angle < fieldOfViewAngle * 0.5f)
        {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, dirToPlayer, out hit))
                return (hit.transform == player);
        }
        return false;
    }

    private void FollowPath()
    {
        if (currentWaypointIndex >= currentPath.Count)
        {
            anim.SetBool("Walk_Anim", false);
            return;
        }
        Vector3 waypoint = currentPath[currentWaypointIndex];
        waypoint.y = transform.position.y;
        float distanceToWaypoint = Vector3.Distance(transform.position, waypoint);
        if (distanceToWaypoint < waypointArrivalThreshold)
        {
            currentWaypointIndex++;
            if (currentWaypointIndex >= currentPath.Count)
            {
                anim.SetBool("Walk_Anim", false);
                return;
            }
            waypoint = currentPath[currentWaypointIndex];
            waypoint.y = transform.position.y;
        }
        float distanceToPlayer = Vector3.Distance(transform.position, player.position);
        if (distanceToPlayer <= stoppingDistance)
        {
            anim.SetBool("Walk_Anim", false);
            return;
        }
        anim.SetBool("Walk_Anim", true);
        Vector3 direction = (waypoint - transform.position).normalized;
        Quaternion targetRotation = Quaternion.LookRotation(direction, Vector3.up);
        transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, rotSpeed * Time.deltaTime);
        transform.position += transform.forward * speed * Time.deltaTime;
    }
}
