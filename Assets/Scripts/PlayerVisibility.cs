using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PlayerVisibility : MonoBehaviour
{
    [Header("Visibility Factors")]
    public float lightLevel = 0.5f; // 0 = dark, 1 = bright
    public float suspicion = 0f;
    public float suspicionIncreaseRate = 10f;
    public float suspicionDecreaseRate = 5f;
    public float maxSuspicion = 100f;

    [Header("Speed Detection")]
    public float movingSpeedThreshold = 0.1f;

    [Header("Debug")]
    public bool debugMode = false;

    private Rigidbody rb;
    private float timeInFOV = 0f;
    private bool isInFOV = false;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        SetInFOV(IsVisibleToClosestGuard());
        UpdateSuspicion();

    }

    public void SetInFOV(bool inFOV)
    {
        isInFOV = inFOV;
        if (!inFOV) timeInFOV = 0f;
    }

    void UpdateSuspicion()
    {
        float speed = rb.velocity.magnitude;
        float movementFactor = speed > movingSpeedThreshold ? 1f : 0.3f;
        float lightFactor = Mathf.Clamp01(lightLevel);
        float distanceFactor = GetClosestGuardDistanceFactor();

        if (isInFOV)
            timeInFOV += Time.deltaTime;

        float fovFactor = Mathf.Clamp01(timeInFOV / 3f); // Max effect after 3s in view

        float suspicionDelta = (lightFactor + movementFactor + distanceFactor + fovFactor) / 4f * suspicionIncreaseRate * Time.deltaTime;

        if (isInFOV)
            suspicion += suspicionDelta;
        else
            suspicion -= suspicionDecreaseRate * Time.deltaTime;

        suspicion = Mathf.Clamp(suspicion, 0, maxSuspicion);

        if (debugMode)
        {
            Debug.Log($"Suspicion: {suspicion:F1} (Light: {lightFactor}, Move: {movementFactor}, Distance: {distanceFactor}, FOV: {fovFactor})");
        }
    }

    bool IsVisibleToClosestGuard()
    {
        GameObject[] guards = GameObject.FindGameObjectsWithTag("Guard");
        float closest = float.MaxValue;
        Transform closestGuard = null;

        foreach (GameObject guard in guards)
        {
            float dist = Vector3.Distance(transform.position, guard.transform.position);
            if (dist < closest)
            {
                closest = dist;
                closestGuard = guard.transform;
            }
        }

        if (closestGuard != null)
        {
            Vector3 dirToPlayer = (transform.position - closestGuard.position).normalized;
            float angle = Vector3.Angle(closestGuard.forward, dirToPlayer);

            float fov = 270f;
            float detectionRange = 15f;

            if (angle < fov * 0.5f)
            {
                RaycastHit hit;
                if (Physics.Raycast(closestGuard.position, dirToPlayer, out hit, detectionRange))
                {
                    return hit.transform == transform;
                }
            }
        }

        return false;
    }


    float GetClosestGuardDistanceFactor()
    {
        GameObject[] guards = GameObject.FindGameObjectsWithTag("Guard");
        float closest = float.MaxValue;

        foreach (GameObject guard in guards)
        {
            Vector3 playerPos = transform.position;
            Vector3 guardPos = guard.transform.position;

            playerPos.y = guardPos.y;
            float dist = Vector3.Distance(playerPos, guardPos);

            if (debugMode)
                Debug.Log($"Distance to {guard.name}: {dist}");

            if (dist < closest)
                closest = dist;
        }

        if (closest < 5f) return 1f;
        else if (closest < 10f) return 0.6f;
        else return 0.2f;
    }


    public float GetSuspicionScore() => suspicion;

    IEnumerator FadeLightLevel(float target, float duration = 0.5f)
    {
        float start = lightLevel;
        float t = 0f;
        while (t < 1f)
        {
            lightLevel = Mathf.Lerp(start, target, t);
            t += Time.deltaTime / duration;
            yield return null;
        }
        lightLevel = target;
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Light"))
            StartCoroutine(FadeLightLevel(1f));
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Light"))
            StartCoroutine(FadeLightLevel(0.2f));
    }
}
