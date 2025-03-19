using UnityEngine;
using TMPro;

public class Battery : MonoBehaviour
{
    public TextMeshProUGUI interactionText;
    private bool inRange = false;

    void Start()
    {
        interactionText.gameObject.SetActive(false);
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Player"))
        {
            interactionText.text = "Press E to pick up battery";
            interactionText.gameObject.SetActive(true);
            inRange = true;
        }
    }

    void OnTriggerStay(Collider other)
    {
        if (inRange && Input.GetKeyDown(KeyCode.E))
        {
            other.GetComponent<PlayerController>().hasBattery = true;
            interactionText.gameObject.SetActive(false);
            Destroy(gameObject);
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Player"))
        {
            interactionText.gameObject.SetActive(false);
            inRange = false;
        }
    }
}