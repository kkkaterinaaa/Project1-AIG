using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GuardController : MonoBehaviour
{
    [Header("Patrol Settings")]
    public Transform[] patrolWaypoints;
    public float patrolSpeed = 2f;
    public float waypointThreshold = 0.7f;
    public float returnCooldownDuration = 5f;

    [Header("Chase Settings")]
    public GuardChase guardChase;

    [Header("Animation")]
    public Animator anim;

    private int currentWaypointIndex = 0;
    private Transform player;
    private Vector3 lastSeenPlayerPosition;
    private bool isChasing = false;
    private bool isReturning = false;
    private float timeOutOfSight = 0f;
    private List<Vector3> currentPath = new List<Vector3>();

    void Start()
    {
        player = GameObject.FindGameObjectWithTag(guardChase.playerTag).transform;
        anim.SetBool("Walk_Anim", true);
        StartPatrolling();
    }

    void Update()
    {
        if (isChasing)
        {
            if (guardChase.IsPlayerVisible())
            {
                timeOutOfSight = 0f;
                lastSeenPlayerPosition = player.position;
            }
            else
            {
                timeOutOfSight += Time.deltaTime;
                if (timeOutOfSight >= returnCooldownDuration)
                {
                    ReturnToPatrol();
                }
            }
        }
        else if (isReturning)
        {
            MoveToNearestWaypoint();
        }
        else
        {
            Patrol();
        }
    }

    void Patrol()
    {
        if (patrolWaypoints.Length == 0) return;

        Vector3 targetPosition = patrolWaypoints[currentWaypointIndex].position;
        targetPosition.y = transform.position.y;

        if (currentPath.Count == 0 || Vector3.Distance(transform.position, targetPosition) <= waypointThreshold)
        {
            currentWaypointIndex = (currentWaypointIndex + 1) % patrolWaypoints.Length;
            targetPosition = patrolWaypoints[currentWaypointIndex].position;
            currentPath = FindDijkstraPath(transform.position, targetPosition);
        }

        if (currentPath.Count > 0)
        {
            FollowPath(currentPath, patrolSpeed);
        }

        if (guardChase.IsPlayerVisible())
        {
            StartChasing();
        }
    }

    void StartChasing()
    {
        isChasing = true;
        anim.SetBool("Walk_Anim", true);
        guardChase.enabled = true;
        guardChase.StartChase();
    }

    void ReturnToPatrol()
    {
        isChasing = false;
        isReturning = true;
        anim.SetBool("Walk_Anim", true);
        guardChase.enabled = false;
        StartCoroutine(ReturnRoutine());
    }

    IEnumerator ReturnRoutine()
    {
        List<Vector3> pathToWaypoint = FindDijkstraPath(transform.position, GetNearestWaypoint());
        if (pathToWaypoint.Count > 0)
        {
            FollowPath(pathToWaypoint, patrolSpeed);
        }
        isReturning = false;
        StartPatrolling();
        yield return null;
    }

    Vector3 GetNearestWaypoint()
    {
        float minDistance = Mathf.Infinity;
        Vector3 nearestWaypoint = Vector3.zero;
        foreach (Transform waypoint in patrolWaypoints)
        {
            float distance = Vector3.Distance(transform.position, waypoint.position);
            if (distance < minDistance)
            {
                minDistance = distance;
                nearestWaypoint = waypoint.position;
            }
        }
        return nearestWaypoint;
    }

    void MoveToNearestWaypoint()
    {
        Vector3 nearestWaypoint = GetNearestWaypoint();
        List<Vector3> pathToWaypoint = FindDijkstraPath(transform.position, nearestWaypoint);
        if (pathToWaypoint.Count > 0)
        {
            FollowPath(pathToWaypoint, patrolSpeed);
        }
        if (Vector3.Distance(transform.position, nearestWaypoint) <= waypointThreshold)
        {
            isReturning = false;
            StartPatrolling();
        }
    }

    List<Vector3> FindDijkstraPath(Vector3 start, Vector3 target)
    {
        Node startNode = guardChase.pathfinder.gridManager.NodeFromWorldPoint(start);
        Node targetNode = guardChase.pathfinder.gridManager.NodeFromWorldPoint(target);

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        startNode.gCost = 0;
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].gCost < currentNode.gCost)
                {
                    currentNode = openSet[i];
                }
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if (currentNode == targetNode)
            {
                return RetracePath(startNode, targetNode);
            }

            foreach (Node neighbour in guardChase.pathfinder.gridManager.GetNeighbours(currentNode))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                    continue;

                int newCostToNeighbour = currentNode.gCost + GetDistance(currentNode, neighbour);
                if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    neighbour.gCost = newCostToNeighbour;
                    neighbour.parent = currentNode;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }
            }
        }
        return new List<Vector3>();
    }

    List<Vector3> RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;
        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse();
        List<Vector3> waypoints = new List<Vector3>();
        foreach (Node node in path)
        {
            waypoints.Add(node.worldPosition);
        }
        return waypoints;
    }

    int GetDistance(Node nodeA, Node nodeB)
    {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        if (dstX > dstY)
            return 14 * dstY + 10 * (dstX - dstY);
        return 14 * dstX + 10 * (dstY - dstX);
    }

    void FollowPath(List<Vector3> path, float speed)
    {
        if (path.Count == 0) return;

        Vector3 target = path[0];
        target.y = transform.position.y;
        Vector3 direction = (target - transform.position).normalized;

        transform.rotation = Quaternion.RotateTowards(transform.rotation, Quaternion.LookRotation(direction), guardChase.rotSpeed * Time.deltaTime);
        transform.position += direction * speed * Time.deltaTime;

        if (Vector3.Distance(transform.position, target) <= waypointThreshold)
        {
            path.RemoveAt(0);
        }
    }

    void MoveTowards(Vector3 target, float speed)
    {
        Vector3 direction = (target - transform.position).normalized;
        transform.rotation = Quaternion.RotateTowards(transform.rotation, Quaternion.LookRotation(direction), guardChase.rotSpeed * Time.deltaTime);
        transform.position += direction * speed * Time.deltaTime;
    }

    void StartPatrolling()
    {
        isChasing = false;
        isReturning = false;
        anim.SetBool("Walk_Anim", true);
        currentPath.Clear();
    }
}
