#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define DHT22_PIN GPIO_NUM_4         // Pino de dados do DHT22
#define LED_PIN GPIO_NUM_2           // Pino do LED 1 (para temperatura)
#define FAN_PIN GPIO_NUM_14          // Pino da ventoinha
#define LED_TIME_PIN GPIO_NUM_18     // Pino do LED 2 (para o tempo)
#define TRIG_PIN GPIO_NUM_15         // Pino TRIG do sensor ultrassônico
#define ECHO_PIN GPIO_NUM_19         // Pino ECHO do sensor ultrassônico
#define PIR_PIN GPIO_NUM_21          // Pino do sensor de presença (PIR)
#define LED_MOTION_PIN GPIO_NUM_22   // Pino do LED para movimento detectado
#define LED_REMINDER_PIN GPIO_NUM_23 // LED para lembrar de manutenção 
#define PUMP_GPIO GPIO_NUM_5         // Pino da Bomba D'água


#define RESERVOIR_HEIGHT 12.5        // Altura do reservatório em cm
#define MAX_VOLUME 1.8               // Capacidade máxima do reservatório em litros
#define TEMP_THRESHOLD 27.0          // Temperatura limite para ligar a ventoinha

static const char *TAG = "DHT22";

// Função para atrasar o tempo em microssegundos
void delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

void weekly_maintenance_task(void *pvParameter) {
    while (1) {
        ESP_LOGI("REMINDER", "Lembrete: Trocar água e nutrientes!");
        gpio_set_level(LED_REMINDER_PIN, 1);  // Acende LED para indicar lembrete
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(LED_REMINDER_PIN, 0);  // Apaga LED
        vTaskDelay(604800000 / portTICK_PERIOD_MS);  // Aguarda 1 semana (604800000 ms)
    }
}

// Função para lembrete de colheita do alface a cada 50 dias
void harvest_task(void *pvParameter) {
    while (1) {
        ESP_LOGI("REMINDER", "Lembrete: Colher o alface!");
        gpio_set_level(LED_REMINDER_PIN, 1);  // Acende LED para indicar lembrete
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(LED_REMINDER_PIN, 0);  // Apaga LED
        vTaskDelay(4320000000 / portTICK_PERIOD_MS);  // Aguarda 50 dias (4320000000 ms)
    }
}

void bomba_dagua_control(bool estado) {
    if (estado) {
        gpio_set_level(PUMP_GPIO, 1); // Liga a Bomba D'água
        printf("Bomba D'água LIGADA\n");
    } else {
        gpio_set_level(PUMP_GPIO, 0); // Desliga a Bomba D'água
        printf("Bomba D'água DESLIGADA\n");
    }
}


// Função para ler a distância do sensor ultrassônico
float read_ultrasonic_distance() {
    gpio_set_level(TRIG_PIN, 0);
    delay_us(2);
    
    gpio_set_level(TRIG_PIN, 1);
    delay_us(10);
    gpio_set_level(TRIG_PIN, 0);
    
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0 && (esp_timer_get_time() - start_time) < 20000);
    
    if (gpio_get_level(ECHO_PIN) == 0) {
        ESP_LOGE(TAG, "Falha na leitura do ECHO (timeout)");
        return -1;
    }
    
    start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1 && (esp_timer_get_time() - start_time) < 20000);
    
    if (gpio_get_level(ECHO_PIN) == 1) {
        ESP_LOGE(TAG, "Falha no fim do pulso do ECHO (timeout)");
        return -1;
    }
    
    int64_t end_time = esp_timer_get_time();
    float duration = (float)(end_time - start_time);
    float distance_cm = (duration / 2.0) * 0.0343;
    return distance_cm;
}

// Função para calcular o volume de água no reservatório
float calculate_water_volume(float distance) {
    float water_height = RESERVOIR_HEIGHT - distance;  
    if (water_height < 0) water_height = 0;
    float volume = (water_height / RESERVOIR_HEIGHT) * MAX_VOLUME;
    return volume;
}

// Função para ler dados reais do DHT22
bool read_dht22(float *temperature, float *humidity) {
    uint8_t data[5] = {0};
    
    gpio_set_direction(DHT22_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT22_PIN, 0);
    delay_us(1000);
    gpio_set_level(DHT22_PIN, 1);
    delay_us(30);
    gpio_set_direction(DHT22_PIN, GPIO_MODE_INPUT);

    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(DHT22_PIN) == 1) {
        if ((esp_timer_get_time() - start_time) > 100) {
            ESP_LOGE(TAG, "Erro de resposta do DHT22");
            return false;
        }
    }

    start_time = esp_timer_get_time();
    while (gpio_get_level(DHT22_PIN) == 0);
    while (gpio_get_level(DHT22_PIN) == 1);

    for (int i = 0; i < 40; i++) {
        while (gpio_get_level(DHT22_PIN) == 0);
        int64_t pulse_start = esp_timer_get_time();
        while (gpio_get_level(DHT22_PIN) == 1);

        if ((esp_timer_get_time() - pulse_start) > 50) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        ESP_LOGE(TAG, "Checksum inválido");
        return false;
    }

    *humidity = ((data[0] << 8) + data[1]) * 0.1;
    *temperature = (((data[2] & 0x7F) << 8) + data[3]) * 0.1;
    if (data[2] & 0x80) *temperature *= -1;

    return true;
}

// Função para controle da ventoinha com base na temperatura
void control_fan(float temperature) {
    if (temperature > TEMP_THRESHOLD) {
        gpio_set_level(FAN_PIN, 1);
        ESP_LOGI(TAG, "Ventoinha LIGADA - Temperatura: %.2f", temperature);
    } else {
        gpio_set_level(FAN_PIN, 0);
        ESP_LOGI(TAG, "Ventoinha DESLIGADA - Temperatura: %.2f", temperature);
    }
}

// Função para verificar a presença com o sensor PIR
void check_motion_sensor() {
    if (gpio_get_level(PIR_PIN) == 1) {
        gpio_set_level(LED_MOTION_PIN, 1);
        ESP_LOGI(TAG, "Movimento detectado!");
    } else {
        gpio_set_level(LED_MOTION_PIN, 0);
    }
}

void app_main(void) {
    float temperature, humidity;
    float distance, volume;

    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(FAN_PIN);
    gpio_set_direction(FAN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_TIME_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(TRIG_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ECHO_PIN);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    
    gpio_reset_pin(PIR_PIN);
    gpio_set_direction(PIR_PIN, GPIO_MODE_INPUT);
    gpio_reset_pin(LED_MOTION_PIN);
    gpio_set_direction(LED_MOTION_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_REMINDER_PIN);
    gpio_set_direction(LED_REMINDER_PIN, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(PUMP_GPIO);
    gpio_set_direction(PUMP_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        bomba_dagua_control(true);  // Liga a Bomba D'água
        vTaskDelay(pdMS_TO_TICKS(5000));  // Espera 5 segundos
        bomba_dagua_control(false); // Desliga a Bomba D'água
        vTaskDelay(pdMS_TO_TICKS(5000));  // Espera mais 5 segundos
    }


    // Inicia as tarefas de lembrete
    xTaskCreate(&weekly_maintenance_task, "Weekly Maintenance Task", 2048, NULL, 1, NULL);
    xTaskCreate(&harvest_task, "Harvest Task", 2048, NULL, 1, NULL);

    printf("Sistema iniciado com lembretes!\n");


    while (1) {
        distance = read_ultrasonic_distance();
        if (distance < 0) {
            ESP_LOGE(TAG, "Erro na leitura do sensor ultrassônico");
        } else {
            printf("Distância medida: %.2f cm\n", distance);
            ESP_LOGI(TAG, "Distância medida: %.2f cm", distance);

            volume = calculate_water_volume(distance);
            ESP_LOGI(TAG, "Volume de água: %.2f L", volume);
        }

        if (read_dht22(&temperature, &humidity)) {
            ESP_LOGI(TAG, "Temperatura: %.2f C, Umidade: %.2f%%", temperature, humidity);
            control_fan(temperature);

            gpio_set_level(LED_PIN, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 0);
        } else {
            ESP_LOGE(TAG, "Erro na leitura do DHT22");
        }

        // Verificar o sensor de presença
        check_motion_sensor();

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
