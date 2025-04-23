using UnityEngine;

public class Projectile : MonoBehaviour
{
    public float lifetime = 5f; 
    public float damage = 10f;
    public GameObject impactEffect;  // Optional impact effect

    private void Start()
    {
        Destroy(gameObject, lifetime);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Player"))
        {
            // If the projectile hits the player, apply damage
            collision.gameObject.GetComponent<HealthPoints>()?.TakeDamage((int)damage);

            if (impactEffect != null)
            {
                Instantiate(impactEffect, transform.position, Quaternion.identity);
            }

            Destroy(gameObject);
        }
        else
        {
            if (impactEffect != null)
            {
                Instantiate(impactEffect, transform.position, Quaternion.identity);
            }

            Destroy(gameObject);
        }
    }
}