using UnityEngine;
using UnityEngine.UI;

public class SuspicionUISlider : MonoBehaviour
{
    public Slider suspicionSlider;
    public PlayerVisibility playerVisibility;

    [Header("Color Settings")]
    public Image fillImage;
    public Color lowColor = Color.green;
    public Color midColor = Color.yellow;
    public Color highColor = Color.red;

    void Start()
    {
        if (playerVisibility == null)
        {
            playerVisibility = GameObject.FindGameObjectWithTag("Player")?.GetComponent<PlayerVisibility>();
        }

        if (suspicionSlider != null)
        {
            suspicionSlider.minValue = 0;
            suspicionSlider.maxValue = playerVisibility.maxSuspicion;
        }
    }

    void Update()
    {
        if (suspicionSlider == null || playerVisibility == null) return;

        float suspicion = playerVisibility.GetSuspicionScore();
        suspicionSlider.value = suspicion;

        float percent = suspicion / playerVisibility.maxSuspicion;
        if (percent < 0.3f)
            fillImage.color = lowColor;
        else if (percent < 0.7f)
            fillImage.color = midColor;
        else
            fillImage.color = highColor;
    }
}
