using UnityEngine;

public class TurretAI : MonoBehaviour
{
    public float detectionRange = 10f;  // Range at which the turret detects the player
    public float fireRate = 1f;  // Fire rate (in seconds)
    public Transform player;  // Reference to the player object
    public bool playerCloaked = false;  // Reference to whether the player is cloaked
    public LayerMask coverLayer;  // Layer for cover objects
    public Transform[] firePoints;  // Array of fire points from which projectiles will be fired
    
    // Projectile Settings
    public GameObject projectilePrefab;  // The projectile prefab to be instantiated
    public float projectileSpeed = 20f;  // Speed of the projectile

    // Rotation Settings
    public float rotationSpeed = 5f;  // Rotation speed of the turret

    private float lastFireTime;

    private void Update()
    {
        if (Time.time - lastFireTime >= fireRate)
        {
            if (IsPlayerInRange() && IsPlayerVisible())
            {
                FireProjectileAtPlayer();
                lastFireTime = Time.time;  
            }
            else if (playerCloaked || IsPlayerBehindCover())
            {
                PauseFiring();
            }
        }

        if (IsPlayerInRange() && IsPlayerVisible())
        {
            RotateTurretTowardsPlayer();
        }
    }

    private bool IsPlayerInRange()
    {
        return Vector3.Distance(transform.position, player.position) <= detectionRange;
    }

    private bool IsPlayerVisible()
    {
        RaycastHit hit;
        Vector3 direction = player.position - transform.position;
        if (Physics.Raycast(transform.position, direction, out hit, detectionRange))
        {
            if (hit.transform == player)
            {
                return true;  
            }
        }
        return false; 
    }

    private bool IsPlayerBehindCover()
    {
        RaycastHit hit;
        Vector3 direction = player.position - transform.position;
        if (Physics.Raycast(transform.position, direction, out hit, detectionRange, coverLayer))
        {
            return hit.transform != player;
        }
        return false;
    }

    private void FireProjectileAtPlayer()
    {
        if (projectilePrefab != null)
        {
            foreach (Transform firePoint in firePoints)
            {
                GameObject projectile = Instantiate(projectilePrefab, firePoint.position, firePoint.rotation);
                Rigidbody rb = projectile.GetComponent<Rigidbody>();

                if (rb != null)
                {
                    Vector3 direction = (player.position - firePoint.position).normalized;
                    rb.velocity = direction * projectileSpeed;
                }

                Debug.Log("Firing projectile from fire point: " + firePoint.name);
            }
        }
        else
        {
            Debug.LogError("Projectile prefab is not assigned!");
        }
    }

    private void PauseFiring()
    {
        Debug.Log("Firing paused, player is cloaked or behind cover");
    }

    private void RotateTurretTowardsPlayer()
    {
        Vector3 direction = player.position - transform.position;
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
    }
}
