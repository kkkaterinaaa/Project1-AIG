using System.Collections;
using UnityEngine;

public class PatrolManager : MonoBehaviour
{
    public GeneticAlgorithm patrolPlanner;
    public AStarPathfinder pathfinderOwner;
    public GuardController[] guards;

    void Start()
    {
        patrolPlanner.StartGA(pathfinderOwner.FindPath);
        StartCoroutine(AssignPathsWhenReady());
    }

    IEnumerator AssignPathsWhenReady()
    {
        while (patrolPlanner.GetBestPaths() == null)
            yield return null;

        var paths = patrolPlanner.GetBestPaths();

        var currentGuards = FindObjectsOfType<GAguard>();

        for (int i = 0; i < currentGuards.Length && i < paths.Count; i++)
        {
            currentGuards[i].SetCustomPatrolPath(paths[i]);
        }
    }

}
