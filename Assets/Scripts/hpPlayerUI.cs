using UnityEngine;
using UnityEngine.UI;

public class hpPlayerUI : MonoBehaviour
{
    public Slider hpSlider;
    public HealthPoints playerHealth;
    public Image fillImage;

    [Header("Color Gradient")]
    public Color fullHealthColor = Color.green;
    public Color midHealthColor = Color.yellow;
    public Color lowHealthColor = Color.red;

    void Start()
    {
        if (playerHealth == null)
        {
            playerHealth = GameObject.FindGameObjectWithTag("Player")?.GetComponent<HealthPoints>();
        }

        if (hpSlider != null && playerHealth != null)
        {
            hpSlider.minValue = 0;
            hpSlider.maxValue = playerHealth.GetMaxHealth();
        }
    }

    void Update()
    {
        if (hpSlider == null || playerHealth == null) return;

        float currentHP = playerHealth.GetCurrentHealth();
        float maxHP = playerHealth.GetMaxHealth();
        float percent = currentHP / maxHP;

        hpSlider.value = currentHP;

        if (percent < 0.15f)
            fillImage.color = lowHealthColor;
        else if (percent < 0.6f)
            fillImage.color = midHealthColor;
        else
            fillImage.color = fullHealthColor;
    }
}
