using UnityEngine;

public class PlayerLaser : MonoBehaviour
{
    public float laserRange = 50f;  // Range of the laser
    public float laserDamage = 10f;  // Damage dealt by the laser
    public float fireRate = 0.1f;  // Fire rate of the laser (in seconds)
    public Material laserMaterial;  // Material for the laser beam

    private float lastFireTime;
    private LineRenderer laserLine;  // The LineRenderer to draw the laser

    private void Start()
    {
        laserLine = gameObject.AddComponent<LineRenderer>();

        laserLine.startWidth = 0.1f;
        laserLine.endWidth = 0.1f;
        laserLine.material = laserMaterial;
        laserLine.positionCount = 2;  
        laserLine.enabled = false;  
    }

    private void Update()
    {
        if (Time.time - lastFireTime >= fireRate && Input.GetKeyDown(KeyCode.F))
        {
            ShootLaser();
            lastFireTime = Time.time;  
        }
        else
        {
            laserLine.enabled = false; 
        }
    }

    private void ShootLaser()
    {
        laserLine.enabled = true;

        laserLine.SetPosition(0, transform.position); 

        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, laserRange))
        {
            laserLine.SetPosition(1, hit.point);
            if (hit.transform.CompareTag("Guard"))
            {
                // Apply damage to the object hit by the laser
                //<gameObject>.GetComponent<HealthPoints>()?.TakeDamage(laserDamage);
            }

            
        }
        else
        {
            // If the laser doesn't hit anything, just set it to the max laser range
            laserLine.SetPosition(1, transform.position + transform.forward * laserRange);  
        }
    }
}
