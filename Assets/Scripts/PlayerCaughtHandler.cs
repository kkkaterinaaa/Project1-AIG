using UnityEngine;
using UnityEngine.UI;

public class PlayerCaughtHandler : MonoBehaviour
{
    [Header("UI Settings")]
    public GameObject caughtScreen; 
    public Transform playerStartPosition;
    public Button retryButton;

    private Transform player;
    public PlayerController playerController;

    void Start()
    {
        player = GameObject.FindGameObjectWithTag("Player").transform;
        caughtScreen.SetActive(false);
        retryButton.onClick.AddListener(RestartGame);
    }

    public void ShowCaughtScreen()
    {
        Debug.Log("Showing caught screen!");
        caughtScreen.SetActive(true);
        Cursor.lockState = CursorLockMode.None;  
        Cursor.visible = true;
        playerController.enabled = false;
        Time.timeScale = 0f;
    }

    public void RestartGame()
    {
        Debug.Log("Before reset, player position: " + player.position);
        caughtScreen.SetActive(false);
        this.transform.position = playerStartPosition.position;
        playerController.enabled = true;
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
        Time.timeScale = 1f;
        Debug.Log("After reset, player position: " + player.position);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Guard") || other.CompareTag("Security"))
        {
            Debug.Log("Player caught!");
            ShowCaughtScreen();
        }
    }
}
