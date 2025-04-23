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
    public float fieldOfViewAngle = 270f;

    private float lastFireTime;

    private PlayerVisibility playerVisibility;

    public float suspicionThreshold = 20f;

    private void Start()
    {
        if (player == null)
            player = GameObject.FindGameObjectWithTag("Player")?.transform;

        if (player != null)
            playerVisibility = player.GetComponent<PlayerVisibility>();
    }

    private void Update()
    {
        if (Time.time - lastFireTime >= fireRate)
        {
            if (IsPlayerInRange() && IsPlayerVisible() && IsSuspicionHighEnough())
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

    public bool IsPlayerVisible()
    {
        Vector3 dirToPlayer = (player.position - transform.position).normalized;
        float angle = Vector3.Angle(transform.forward, dirToPlayer);

        if (angle < fieldOfViewAngle * 0.5f)
        {
            RaycastHit hit;
            if (Physics.Raycast(transform.position, dirToPlayer, out hit, detectionRange))
            {
                return hit.transform == player;
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
    }

    private void RotateTurretTowardsPlayer()
    {
        Vector3 direction = player.position - transform.position;
        Quaternion targetRotation = Quaternion.LookRotation(direction);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
    }

    private bool IsSuspicionHighEnough()
    {
        if (playerVisibility == null) return false;
        return playerVisibility.GetSuspicionScore() >= suspicionThreshold;
    }
}
