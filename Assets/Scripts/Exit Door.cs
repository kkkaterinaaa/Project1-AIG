using UnityEngine;
using TMPro;

public class ExitDoor : MonoBehaviour
{
    public TextMeshProUGUI interactionText;
    public GameObject escapeScreen;
    private bool inRange = false;

    void Start()
    {
        interactionText.gameObject.SetActive(false);
        escapeScreen.SetActive(false);
    }

    void Update()
    {
        if (inRange)
        {
            bool hasBattery = GameObject.FindWithTag("Player").GetComponent<PlayerController>().hasBattery;
            interactionText.text = hasBattery ? "Press E to escape" : "I need to find the battery first";
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Player"))
        {
            interactionText.gameObject.SetActive(true);
            inRange = true;
        }
    }

    void OnTriggerStay(Collider other)
    {
        if (inRange && Input.GetKeyDown(KeyCode.E))
        {
            bool hasBattery = other.GetComponent<PlayerController>().hasBattery;
            if (hasBattery)
            {
                Time.timeScale = 0f;
                escapeScreen.SetActive(true);
                Cursor.lockState = CursorLockMode.None;
                Cursor.visible = true;
            }
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