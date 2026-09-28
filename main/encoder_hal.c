#include "driver/gpio.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#define ENC_A_GPIO     31
#define ENC_B_GPIO     32
#define ENC_SW_GPIO    33

static char *TAG = "ENCODER";

static volatile int32_t encoder_diff = 0;
static volatile bool button_state = false;

static int last_a = 0;
static int last_b = 0;

static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    int a = gpio_get_level(ENC_A_GPIO);
    int b = gpio_get_level(ENC_B_GPIO);

    if (a != last_a) {
        if (a == b) encoder_diff++;
        else encoder_diff--;
    }

    last_a = a;
    last_b = b;

    ESP_LOGI(TAG, "ENCODER= %d" , encoder_diff);
}

static void IRAM_ATTR button_isr_handler(void *arg)
{
    button_state = !gpio_get_level(ENC_SW_GPIO);
}

/* ------------------------------ */
/*      API PARA LVGL            */
/* ------------------------------ */

int32_t hal_encoder_get_diff(void)
{
    int32_t diff = encoder_diff;
    encoder_diff = 0;      // limpiar después de leer
    return diff;
}

bool hal_encoder_is_pressed(void)
{
    return button_state;
}

/* ------------------------------ */
/*  INICIALIZACIÓN DEL ENCODER   */
/* ------------------------------ */

void hal_encoder_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENC_A_GPIO) | (1ULL << ENC_B_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(ENC_A_GPIO, encoder_isr_handler, NULL);
    gpio_isr_handler_add(ENC_B_GPIO, encoder_isr_handler, NULL);

    /* Botón del encoder */
    gpio_config_t btn_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENC_SW_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn_conf);

    gpio_isr_handler_add(ENC_SW_GPIO, button_isr_handler, NULL);

    last_a = gpio_get_level(ENC_A_GPIO);
    last_b = gpio_get_level(ENC_B_GPIO);
}
