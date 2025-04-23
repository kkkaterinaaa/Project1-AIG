using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class GeneticAlgorithm : MonoBehaviour
{
    [Header("Waypoints and Guards")]
    public Transform[] allWaypoints;
    public List<GameObject> guards = new List<GameObject>();

    [Header("Grid and Pathfinding")]
    public GridManager gridManager;
    public MonoBehaviour pathfinderOwner;

    [Header("GA Settings")]
    public int populationSize = 30;
    public int generations = 100;
    public float mutationRate = 0.1f;
    public float delayBetweenGenerations = 0.1f;

    public delegate List<Vector3> PathfindingDelegate(Vector3 start, Vector3 end);
    private PathfindingDelegate FindPath;

    private List<List<Vector3>> bestPaths;

    public void StartGA(PathfindingDelegate pathfinder)
    {
        if (guards.Count == 0 || allWaypoints.Length == 0)
        {
            Debug.LogError("Guards or waypoints are missing. Please assign them in the inspector.");
            return;
        }

        FindPath = pathfinder;

        Debug.Log($"[GA] Starting Genetic Algorithm for {guards.Count} guards and {allWaypoints.Length} waypoints.");

        // Ensure guards are valid
        for (int i = 0; i < guards.Count; i++)
        {
            if (guards[i] == null)
                Debug.LogWarning($"Guard {i} is null!");
            else
                Debug.Log($"Guard {i}: {guards[i].name}");
        }

        StartCoroutine(RunGA());
    }



    public void Replan()
    {
        StopAllCoroutines();
        StartGA(FindPath);
    }

    public List<List<Vector3>> GetBestPaths() => bestPaths;

    public void RemoveGuard(GameObject guard)
    {
        if (guards.Contains(guard))
        {
            guards.Remove(guard);
            Replan();
        }
    }

    IEnumerator RunGA()
    {
        var population = GenerateInitialPopulation();

        for (int gen = 0; gen < generations; gen++)
        {
            var evaluated = population
                .Select(chromo => (chromo, fitness: EvaluateFitness(chromo)))
                .OrderByDescending(x => x.fitness)
                .ToList();

            var newPopulation = new List<List<List<int>>> { evaluated[0].chromo };

            while (newPopulation.Count < populationSize)
            {
                var p1 = TournamentSelection(evaluated, 5);
                var p2 = TournamentSelection(evaluated, 5);
                var child = Crossover(p1, p2);
                Mutate(child);
                newPopulation.Add(child);
            }

            population = newPopulation;

            Debug.Log($"Generation {gen + 1}/{generations}, Best Fitness: {evaluated[0].fitness}");
            yield return new WaitForSeconds(delayBetweenGenerations);
        }

        bestPaths = ConvertToWorldPaths(population[0]);
    }

    List<List<List<int>>> GenerateInitialPopulation()
    {
        List<List<List<int>>> population = new List<List<List<int>>>();
        int wpCount = allWaypoints.Length;
        int guardCount = guards.Count;

        for (int i = 0; i < populationSize; i++)
        {
            var indices = Enumerable.Range(0, wpCount).OrderBy(_ => Random.value).ToList();
            var assignment = new List<List<int>>();
            int perGuard = Mathf.CeilToInt((float)wpCount / guardCount);

            for (int g = 0; g < guardCount; g++)
            {
                var chunk = indices.Skip(g * perGuard).Take(perGuard).ToList();
                assignment.Add(chunk);
            }
            population.Add(assignment);
        }
        return population;
    }

    float EvaluateFitness(List<List<int>> chromo)
    {
        Dictionary<Node, int> visitCounts = new Dictionary<Node, int>();
        HashSet<int> visitedWaypoints = new HashSet<int>();

        for (int g = 0; g < chromo.Count; g++)
        {
            var wpIndices = chromo[g];

            if (wpIndices.Count == 0)
            {
                Debug.LogWarning($"[GA] Guard {g} has no assigned waypoints.");
                continue;
            }

            visitedWaypoints.UnionWith(wpIndices);

            Vector3 currentPos = allWaypoints[wpIndices[0]].position;

            for (int i = 1; i < wpIndices.Count; i++)
            {
                Vector3 nextPos = allWaypoints[wpIndices[i]].position;
                var path = FindPath(currentPos, nextPos);

                if (path == null || path.Count == 0)
                {
                    Debug.LogWarning($"[GA] Invalid path from WP {wpIndices[i - 1]} to {wpIndices[i]} (Guard {g}).");
                    return -9999f;
                }

                foreach (var point in path)
                {
                    var node = gridManager.NodeFromWorldPoint(point);
                    if (visitCounts.ContainsKey(node)) visitCounts[node]++;
                    else visitCounts[node] = 1;
                }

                currentPos = nextPos;
            }
        }

        float uniqueCoverage = visitCounts.Keys.Count;
        float overlapPenalty = visitCounts.Values.Where(v => v > 1).Sum(v => v - 1);
        float completenessScore = (visitedWaypoints.Count == allWaypoints.Length) ? 1000f : -1000f;

        if (visitedWaypoints.Count != allWaypoints.Length)
            Debug.LogWarning($"[GA] Chromosome missing some waypoints. Visited {visitedWaypoints.Count} / {allWaypoints.Length}");

        return uniqueCoverage - overlapPenalty + completenessScore;
    }


    List<List<int>> TournamentSelection(List<(List<List<int>>, float)> population, int size)
    {
        return population.OrderBy(_ => Random.value).Take(size).OrderByDescending(x => x.Item2).First().Item1;
    }

    List<List<int>> Crossover(List<List<int>> p1, List<List<int>> p2)
    {
        int guardCount = p1.Count;
        var child = new List<List<int>>();
        var allIndices = new HashSet<int>();

        for (int i = 0; i < guardCount; i++)
        {
            var chosen = Random.value < 0.5f ? p1[i] : p2[i];
            child.Add(new List<int>(chosen));
            allIndices.UnionWith(chosen);
        }

        var missing = Enumerable.Range(0, allWaypoints.Length).Except(allIndices).ToList();
        foreach (var m in missing)
            child[Random.Range(0, guardCount)].Add(m);

        return child;
    }

    void Mutate(List<List<int>> chromo)
    {
        if (Random.value > mutationRate) return;

        int g1 = Random.Range(0, chromo.Count);
        int g2 = Random.Range(0, chromo.Count);

        if (chromo[g1].Count == 0 || chromo[g2].Count == 0) return;

        int i1 = Random.Range(0, chromo[g1].Count);
        int i2 = Random.Range(0, chromo[g2].Count);

        (chromo[g1][i1], chromo[g2][i2]) = (chromo[g2][i2], chromo[g1][i1]);
    }

    List<List<Vector3>> ConvertToWorldPaths(List<List<int>> chromosome)
    {
        List<List<Vector3>> worldPaths = new List<List<Vector3>>();

        for (int g = 0; g < chromosome.Count; g++)
        {
            try
            {
                var wpIndices = chromosome[g];

                if (wpIndices.Count == 0)
                {
                    Debug.LogWarning($"[GA] No path for Guard {g}.");
                    worldPaths.Add(new List<Vector3>());
                    continue;
                }

                if (guards[g] == null)
                {
                    Debug.LogError($"[GA] Guard {g} is null during path conversion.");
                    continue;
                }

                var guardPos = guards[g].transform.position;
                int closestIdx = wpIndices.OrderBy(i => Vector3.Distance(guardPos, allWaypoints[i].position)).First();
                int startIdx = wpIndices.IndexOf(closestIdx);
                var reordered = wpIndices.Skip(startIdx).Concat(wpIndices.Take(startIdx)).ToList();

                List<Vector3> fullPath = FindPath(guardPos, allWaypoints[reordered[0]].position);
                Debug.Log($"[GA] Guard {g} starting at WP {reordered[0]}");

                for (int i = 0; i < reordered.Count - 1; i++)
                {
                    var from = allWaypoints[reordered[i]].position;
                    var to = allWaypoints[reordered[i + 1]].position;
                    var segment = FindPath(from, to);
                    if (segment == null || segment.Count == 0)
                    {
                        Debug.LogWarning($"[GA] Segment failed from WP {reordered[i]} to {reordered[i + 1]}");
                        continue;
                    }
                    fullPath.AddRange(segment);
                }

                worldPaths.Add(fullPath);
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"[GA] Error building path for guard {g}: {ex.Message}");
            }
        }

        return worldPaths;
    }


    void OnDrawGizmos()
    {
        if (bestPaths == null) return;

        Color[] colors = { Color.green, Color.cyan, Color.magenta, Color.yellow, Color.red, Color.blue };
        for (int i = 0; i < bestPaths.Count; i++)
        {
            Gizmos.color = colors[i % colors.Length];
            var path = bestPaths[i];
            for (int j = 0; j < path.Count - 1; j++)
            {
                Gizmos.DrawLine(path[j], path[j + 1]);
                Gizmos.DrawSphere(path[j], 0.15f);
            }
        }
    }
}
