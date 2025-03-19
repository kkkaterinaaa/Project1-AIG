using UnityEngine;

public class PlayerController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float speed = 6f;
    public float gravity = -9.81f;

    [Header("Mouse Look Settings")]
    public float mouseSensitivity = 2f;
    public float verticalRotationLimit = 80f;
    
    private CharacterController controller;
    private Camera playerCamera;
    private float verticalVelocity = 0f;
    private float verticalRotation = 0f;

    [HideInInspector] public bool hasBattery = false;

    void Start()
    {
        controller = GetComponent<CharacterController>();
        playerCamera = Camera.main;
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    void Update()
    {
        HandleMovement();
        HandleMouseLook();
    }
    
    void HandleMovement()
    {
        // Get input from WASD or arrow keys.
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");
        Vector3 move = transform.right * horizontal + transform.forward * vertical;
        if (controller.isGrounded && verticalVelocity < 0)
        {
            verticalVelocity = 0f;
        }
        verticalVelocity += gravity * Time.deltaTime;
        move.y = verticalVelocity;
        controller.Move(move * speed * Time.deltaTime);
    }
    
    void HandleMouseLook()
    {
        float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity;
        transform.Rotate(0, mouseX, 0);
        float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity;
        verticalRotation -= mouseY;
        verticalRotation = Mathf.Clamp(verticalRotation, -verticalRotationLimit, verticalRotationLimit);
        playerCamera.transform.localRotation = Quaternion.Euler(verticalRotation, 0, 0);
    }
}
