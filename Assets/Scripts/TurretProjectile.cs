using UnityEngine;

public class Projectile : MonoBehaviour
{
    public float lifetime = 5f;  // Time before the projectile is destroyed if it doesn't hit anything
    public float damage = 10f;  // Damage dealt by the projectile
    public GameObject impactEffect;  // Optional impact effect when the projectile hits something

    private void Start()
    {
        Destroy(gameObject, lifetime);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Player"))
        {
            // If the projectile hits the player, apply damage (you can add your own damage logic here)
            collision.gameObject.GetComponent<HealthPoints>()?.TakeDamage((int)damage);

            // Optionally, instantiate an impact effect
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