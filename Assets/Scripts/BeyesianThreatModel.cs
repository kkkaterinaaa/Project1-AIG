using UnityEngine;

public class BayesianThreatModel : MonoBehaviour
{
    [Header("Bayesian Settings")]
    [Range(0f, 1f)] public float priorThreatProbability = 0.1f;
    public float thresholdToChase = 0.8f;

    [Header("Confidence Tuning")]
    public float minLikelihood = 0.01f; // soft floor to avoid zero
    public float beliefSmoothing = 0.15f; // dampens quick jumps
    public float decayRate = 0.05f; // slower decay

    [Header("Debug")]
    public bool debug = false;

    private float currentThreat;

    private void Awake()
    {
        currentThreat = priorThreatProbability;
    }

    public void UpdateBelief(ThreatEvidence evidence)
    {
        float prior = currentThreat;

        float numerator = evidence.likelihoodIfThreat * prior;
        float denominator = numerator + evidence.likelihoodIfInnocent * (1f - prior);
        float posterior = (denominator > 0f) ? numerator / denominator : prior;

        currentThreat = Mathf.Lerp(currentThreat, posterior, beliefSmoothing);
        currentThreat = Mathf.Clamp01(currentThreat);
        float delta = Mathf.Abs(currentThreat - prior);
        if (debug && delta > 0.01f)
        {
            Debug.Log($"[Bayes] {name} <- {evidence.source} | P(T): {prior:F2} -> {currentThreat:F2} | L(E|T): {evidence.likelihoodIfThreat:F2}, L(E|-T): {evidence.likelihoodIfInnocent:F2}");
        }
    }


    public void DecayThreat()
    {
        currentThreat = Mathf.Lerp(currentThreat, priorThreatProbability, decayRate * Time.deltaTime);
    }

    public void ApplyStrongEvidence(float newBelief, string source = "Critical")
    {
        float prior = currentThreat;
        currentThreat = Mathf.Clamp01(newBelief);

        if (debug)
        {
            Debug.Log($"[Bayes] {name} <- {source} (FORCE) | P(T): {prior:F2} -> {currentThreat:F2}");
        }
    }

    public float GetThreatLevel() => currentThreat;

    public bool IsThreatening() => currentThreat >= thresholdToChase;
}

public struct ThreatEvidence
{
    public float likelihoodIfThreat;
    public float likelihoodIfInnocent;
    public string source;

    public ThreatEvidence(float threat, float innocent, string source)
    {
        likelihoodIfThreat = Mathf.Clamp01(threat);
        likelihoodIfInnocent = Mathf.Clamp01(innocent);
        this.source = source;
    }
}
