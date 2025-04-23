using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class HealthPoints : MonoBehaviour
{
    [Header("Health Settings")]
    public int maxHealth = 100;
    private int currentHealth;

    [Header("Entity Type")]
    public bool isEnemy = true;

    private PlayerCaughtHandler playerCaughtHandler;

    void Start()
    {
        currentHealth = maxHealth;

        if (!isEnemy)
        {
            playerCaughtHandler = FindObjectOfType<PlayerCaughtHandler>();
            if (playerCaughtHandler == null)
            {
                Debug.LogWarning("PlayerCaughtHandler not found in scene!");
            }
        }
    }

    public void TakeDamage(int damage)
    {
        currentHealth -= damage;
        currentHealth = Mathf.Clamp(currentHealth, 0, maxHealth);

        Debug.Log($"{gameObject.name} took {damage} damage. Current HP: {currentHealth}");

        if (currentHealth <= 0)
        {
            Die();
        }
    }

    private void Die()
    {
        if (isEnemy)
        {
            Debug.Log($"{gameObject.name} (Enemy) died!");
            GameObject corpse = new GameObject("DeadGuardMarker");
            corpse.transform.position = transform.position;
            corpse.tag = "DeadGuard";
            var model = GetComponent<BayesianThreatModel>();
            model.ApplyStrongEvidence(0.7f, "Murder");
            GAguard guard = GetComponent<GAguard>();
            if (guard != null && guard.patrolPlanner != null)
            {
                Debug.Log($"[GA] Notifying GeneticAlgorithm to remove {gameObject.name}.");
                guard.patrolPlanner.RemoveGuard(gameObject);
            }
            else
            {
                Debug.LogWarning($"[GA] No GuardController or planner found on {gameObject.name}.");
            }

            StartCoroutine(DelayedDestroy());
        }
        else
        {
            Debug.Log($"{gameObject.name} (Player) died!");
            if (playerCaughtHandler != null)
            {
                playerCaughtHandler.ShowCaughtScreen();
            }
            else
            {
                Debug.LogWarning("Cannot show caught screen, PlayerCaughtHandler not assigned.");
            }
        }
    }

    public void Heal(int amount)
    {
        currentHealth += amount;
        currentHealth = Mathf.Clamp(currentHealth, 0, maxHealth);
        Debug.Log($"{gameObject.name} healed {amount}. Current HP: {currentHealth}");
    }

    public int GetCurrentHealth() => currentHealth;
    public int GetMaxHealth() => maxHealth;

    IEnumerator DelayedDestroy()
    {
        yield return new WaitForSeconds(0.1f); 
        Destroy(gameObject);
    }

}
