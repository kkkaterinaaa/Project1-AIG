using System.Collections.Generic;
using UnityEngine;

public class AStarPathfinder : MonoBehaviour
{
    public GridManager gridManager;
    
    void Awake(){
        gridManager = FindObjectOfType<GridManager>();
    }
    
    public List<Vector3> FindPath(Vector3 startPos, Vector3 targetPos){
        Node startNode = gridManager.NodeFromWorldPoint(startPos);
        Node targetNode = gridManager.NodeFromWorldPoint(targetPos);
        
        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        startNode.gCost = 0;
        startNode.hCost = GetDistance(startNode, targetNode);
        openSet.Add(startNode);
        
        while (openSet.Count > 0){
            Node currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++){
                if (openSet[i].fCost < currentNode.fCost || 
                   (openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost))
                {
                    currentNode = openSet[i];
                }
            }
            
            openSet.Remove(currentNode);
            closedSet.Add(currentNode);
            if (currentNode == targetNode)
                return RetracePath(startNode, targetNode);
            
            foreach (Node neighbour in gridManager.GetNeighbours(currentNode)){
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                    continue;
                
                int newMovementCostToNeighbour = currentNode.gCost + GetDistance(currentNode, neighbour);
                if (newMovementCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)){
                    neighbour.gCost = newMovementCostToNeighbour;
                    neighbour.hCost = GetDistance(neighbour, targetNode);
                    neighbour.parent = currentNode;
                    
                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }
            }
        }
        return new List<Vector3>();
    }
    List<Vector3> RetracePath(Node startNode, Node endNode){
        List<Node> path = new List<Node>();
        Node currentNode = endNode;
        
        while (currentNode != startNode){
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Reverse();
        List<Vector3> waypoints = new List<Vector3>();
        foreach (Node node in path){
            waypoints.Add(node.worldPosition);
        }
        return waypoints;
    }
    int GetDistance(Node nodeA, Node nodeB){
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
        
        if (dstX > dstY)
            return 14 * dstY + 10 * (dstX - dstY);
        return 14 * dstX + 10 * (dstY - dstX);
    }
}
