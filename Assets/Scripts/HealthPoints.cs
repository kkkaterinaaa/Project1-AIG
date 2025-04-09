using UnityEngine;

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
            Destroy(gameObject);
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
}
