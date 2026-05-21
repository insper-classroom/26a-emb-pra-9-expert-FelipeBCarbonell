#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pins.h"
#include "mpu6050.h"
#include "Fusion.h"

// =============================================================================
//  Configurações gerais
// =============================================================================

#define SAMPLE_PERIOD       (0.01f)   // período de amostragem em segundos (100 Hz)
#define SAMPLE_PERIOD_MS    10        // equivalente em ms para vTaskDelay

// I2C – pinos do PICO DOCK (GPIO 16 = SDA, GPIO 17 = SCL para i2c0)
#define MPU_ADDRESS     0x68
#define I2C_SDA_GPIO    17
#define I2C_SCL_GPIO    16

// Protocolo UART binário: [0xFF, axis, value+128]
#define SYNC_BYTE       0xFF
#define VALUE_OFFSET    128
#define AXIS_X          0     // eixo X do mouse
#define AXIS_Y          1     // eixo Y do mouse
#define AXIS_CLICK      2     // evento de clique
#define MAX_ABS         95    // valor máximo em módulo enviado ao Python

// Detecção de clique (gesto "cutucada no ar")
// Ajuste CLICK_THRESHOLD_G conforme a sensibilidade desejada (2.2g é um bom ponto de partida)
#define CLICK_THRESHOLD_G   1.5f   // aceleração de pico em g para disparar o clique
#define CLICK_COOLDOWN_MS   600    // tempo mínimo entre dois cliques (ms)

// Ângulo máximo mapeado para cor total (graus)
#define MAX_ANGLE   90.0f

// =============================================================================
//  Tipos de dados compartilhados entre tasks
// =============================================================================

typedef struct {
    int16_t accel[3];   // aceleração bruta (X, Y, Z)
    int16_t gyro[3];    // giroscópio bruto (X, Y, Z)
} imu_data_t;

typedef struct {
    int8_t x;           // deslocamento do mouse em X  [-95, +95]
    int8_t y;           // deslocamento do mouse em Y  [-95, +95]
} pos_data_t;

typedef struct {
    uint8_t r;          // canal vermelho  [0, 80]
    uint8_t g;          // canal verde     [0, 80]
    uint8_t b;          // canal azul      [0, 80]
} color_data_t;

// =============================================================================
//  Objetos FreeRTOS (filas e semáforo)
// =============================================================================

QueueHandle_t     xQueueMPU;       // dados brutos IMU  → fusion_task
QueueHandle_t     xQueuePos;       // posição do mouse  → uart_task
QueueHandle_t     xQueueColor;     // cor RGB           → led_task
SemaphoreHandle_t xSemaphoreBtn;   // evento de clique  → uart_task

// =============================================================================
//  Driver MPU6050 (I2C)
// =============================================================================

static void mpu6050_hw_init(void) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Acorda o MPU6050 (limpa o bit SLEEP no registrador PWR_MGMT_1)
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 14, false);

    // 0x3B-0x40: aceleração (6 bytes)
    for (int i = 0; i < 3; i++)
        accel[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);

    // 0x41-0x42: temperatura (2 bytes)
    *temp = (int16_t)((buffer[6] << 8) | buffer[7]);

    // 0x43-0x48: giroscópio (6 bytes)
    for (int i = 0; i < 3; i++)
        gyro[i] = (int16_t)((buffer[8 + i * 2] << 8) | buffer[8 + i * 2 + 1]);
}

// =============================================================================
//  Utilitários
// =============================================================================

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void pwm_pin_init(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, 255);
    pwm_set_enabled(slice, true);
}

static void pwm_set_level(uint gpio, uint8_t level) {
    pwm_set_chan_level(
        pwm_gpio_to_slice_num(gpio),
        pwm_gpio_to_channel(gpio),
        level
    );
}

static void debug_gpio_init(uint gpio) {
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 0);
}

// =============================================================================
//  Task: mpu_task
//  Lê dados brutos da IMU a 100 Hz e envia para xQueueMPU.
// =============================================================================

void mpu_task(void *p) {
    mpu6050_hw_init();
    debug_gpio_init(MPU);

    imu_data_t data;
    int16_t temp;

    while (true) {
        gpio_put(MPU, 1);
        mpu6050_read_raw(data.accel, data.gyro, &temp);
        xQueueSend(xQueueMPU, &data, 0);
        gpio_put(MPU, 0);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// =============================================================================
//  Task: fusion_task
//  Executa o algoritmo AHRS (Fusion), detecta o gesto de clique,
//  mapeia orientação para posição do mouse e cor do LED.
// =============================================================================

void fusion_task(void *p) {
    // Inicializa pino de temporização para osciloscópio (baixo por padrão)
    debug_gpio_init(FUSION);

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    TickType_t last_click   = 0;
    bool       gesture_armed = true;  // pronto para detectar o próximo clique

    imu_data_t data;

    while (true) {
        if (xQueueReceive(xQueueMPU, &data, portMAX_DELAY) != pdTRUE)
            continue;

        // Marca início da iteração (medir do início ao fim da iteração)
        gpio_put(FUSION, 1);

        // ── Fusão de sensores ─────────────────────────────────────────────────
        FusionVector gyroscope = {
            .axis.x = data.gyro[0] / 131.0f,   // conversão para °/s (±250°/s full-scale)
            .axis.y = data.gyro[1] / 131.0f,
            .axis.z = data.gyro[2] / 131.0f,
        };
        FusionVector accelerometer = {
            .axis.x = data.accel[0] / 16384.0f, // conversão para g  (±2g full-scale)
            .axis.y = data.accel[1] / 16384.0f,
            .axis.z = data.accel[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler =
            FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        float roll  = euler.angle.roll;   // inclinação esquerda/direita
        float pitch = euler.angle.pitch;  // inclinação frente/trás

        // ── Detecção de clique (gesto "cutucada no ar") ───────────────────────
        // Monitora o eixo Y da aceleração (eixo "para frente" quando o controle
        // é segurado na horizontal). Um pico acima de CLICK_THRESHOLD_G indica
        // uma cutucada rápida no ar.
        // DICA: se o clique não disparar, reduza CLICK_THRESHOLD_G.
        //       Se disparar acidentalmente, aumente-o.
        float ay = data.accel[1] / 16384.0f;
        TickType_t now = xTaskGetTickCount();

        if (gesture_armed && fabsf(ay) > CLICK_THRESHOLD_G) {
            if ((now - last_click) > pdMS_TO_TICKS(CLICK_COOLDOWN_MS)) {
                xSemaphoreGive(xSemaphoreBtn);
                last_click    = now;
                gesture_armed = false;  // requer que a aceleração caia antes do próximo clique
            }
        } else if (!gesture_armed && fabsf(ay) < 0.5f) {
            gesture_armed = true;       // rearmado quando a aceleração volta ao repouso
        }

        // ── Posição do mouse ──────────────────────────────────────────────────
        // roll  → eixo X (inclinação direita = mouse para direita)
        // pitch → eixo Y (inclinação para frente = mouse para cima)
        // Escala: ±45° mapeia para ±MAX_ABS
        float scale = (float)MAX_ABS / 45.0f;
        int8_t mx = (int8_t)clampf(-roll  * scale, -MAX_ABS, MAX_ABS);
        int8_t my = (int8_t)clampf(pitch * scale, -MAX_ABS, MAX_ABS);

        pos_data_t pos = {mx, my};
        xQueueOverwrite(xQueuePos, &pos);  // sobrescreve: sempre o valor mais recente

        // ── Cor do LED RGB ────────────────────────────────────────────────────
        // roll < 0  (esquerda) → azul
        // roll > 0  (direita)  → vermelho
        // pitch < 0 (frente)   → verde
        color_data_t col = {0, 0, 0};
        if (roll  < 0) col.b = (uint8_t)clampf(-roll  * (40.0f / MAX_ANGLE), 0.0f, 40.0f);
        if (roll  > 0) col.r = (uint8_t)clampf( roll  * (40.0f / MAX_ANGLE), 0.0f, 40.0f);
        if (pitch < 0) col.g = (uint8_t)clampf(-pitch * (40.0f / MAX_ANGLE), 0.0f, 40.0f);

        xQueueOverwrite(xQueueColor, &col);

        // Marca fim da iteração de processamento
        gpio_put(FUSION, 0);
    }
}

// =============================================================================
//  Task: uart_task
//  Envia pacotes de posição e evento de clique pela UART/USB-CDC.
//  Protocolo: [0xFF, axis, value+128]
// =============================================================================

void uart_task(void *p) {
    debug_gpio_init(UART);

    while (true) {
        pos_data_t pos;

        // Envia posição X e Y a cada novo dado disponível
        if (xQueueReceive(xQueuePos, &pos, pdMS_TO_TICKS(15)) == pdTRUE) {
            gpio_put(UART, 1);
            uint8_t pkt[3];

            // Pacote X
            pkt[0] = SYNC_BYTE;
            pkt[1] = AXIS_X;
            pkt[2] = (uint8_t)(pos.x + VALUE_OFFSET);
            for (int i = 0; i < 3; i++) putchar_raw(pkt[i]);

            // Pacote Y
            pkt[0] = SYNC_BYTE;
            pkt[1] = AXIS_Y;
            pkt[2] = (uint8_t)(pos.y + VALUE_OFFSET);
            for (int i = 0; i < 3; i++) putchar_raw(pkt[i]);
            gpio_put(UART, 0);
        }

        // Envia pacote de clique se o semáforo foi dado
        if (xSemaphoreTake(xSemaphoreBtn, 0) == pdTRUE) {
            gpio_put(UART, 1);
            uint8_t pkt_click[3] = {SYNC_BYTE, AXIS_CLICK, VALUE_OFFSET};
            for (int i = 0; i < 3; i++) putchar_raw(pkt_click[i]);
            gpio_put(UART, 0);
        }
    }
}

// =============================================================================
//  Task: led_task
//  Recebe cor da fila e aciona o LED RGB via PWM com suavização EMA.
// =============================================================================

void led_task(void *p) {
    pwm_pin_init(LED_PIN_R);
    pwm_pin_init(LED_PIN_G);
    pwm_pin_init(LED_PIN_B);
    debug_gpio_init(LED_PWM);

    // Média móvel exponencial (EMA) para suavizar transições de cor
    // alpha próximo de 0 → transição lenta; próximo de 1 → transição rápida
    float sr = 0.0f, sg = 0.0f, sb = 0.0f;
    const float alpha = 0.12f;

    while (true) {
        color_data_t col;
        if (xQueueReceive(xQueueColor, &col, pdMS_TO_TICKS(20)) == pdTRUE) {
            gpio_put(LED_PWM, 1);
            sr = alpha * (float)col.r + (1.0f - alpha) * sr;
            sg = alpha * (float)col.g + (1.0f - alpha) * sg;
            sb = alpha * (float)col.b + (1.0f - alpha) * sb;

            pwm_set_level(LED_PIN_R, (uint8_t)sr);
            pwm_set_level(LED_PIN_G, (uint8_t)sg);
            pwm_set_level(LED_PIN_B, (uint8_t)sb);
            gpio_put(LED_PWM, 0);
        }
    }
}

// =============================================================================
//  Stack monitor
// =============================================================================

typedef struct {
    TaskHandle_t  h;
    const char   *name;
    UBaseType_t   total_words;
} stack_info_t;

void stack_monitor_task(void *p) {
    stack_info_t *inf = (stack_info_t *)p;
    vTaskDelay(pdMS_TO_TICKS(2000));  // aguarda tasks estabilizarem
    while (true) {
        printf("\n=== Stack Usage ===\n");
        for (int i = 0; i < 4; i++) {
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(inf[i].h);
            UBaseType_t used_words = inf[i].total_words - free_words;
            unsigned pct = (unsigned)((used_words * 100u) / inf[i].total_words);
            printf("%-12s  used=%4u/%4u words  (%u%%)\n",
                   inf[i].name,
                   (unsigned)used_words,
                   (unsigned)inf[i].total_words,
                   pct);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// =============================================================================
//  main
// =============================================================================

int main(void) {
    stdio_init_all();

    // Cria as filas (tamanho 1 para pos e color permite uso de xQueueOverwrite)
    xQueueMPU     = xQueueCreate(4, sizeof(imu_data_t));
    xQueuePos     = xQueueCreate(1, sizeof(pos_data_t));
    xQueueColor   = xQueueCreate(1, sizeof(color_data_t));
    xSemaphoreBtn = xSemaphoreCreateBinary();

    // Cria as tasks
    // Prioridade 2 para mpu/fusion (tempo-real), 1 para uart/led (melhor-esforço)
    TaskHandle_t h_mpu, h_fusion, h_uart, h_led;
    xTaskCreate(mpu_task,    "MPU",    8192, NULL, 2, &h_mpu);
    xTaskCreate(fusion_task, "FUSION", 8192, NULL, 2, &h_fusion);
    xTaskCreate(uart_task,   "UART",   4096, NULL, 1, &h_uart);
    xTaskCreate(led_task,    "LED",    4096, NULL, 1, &h_led);

    // Stack monitor: imprime uso de stack de cada task a cada 5 s pela serial
    // Leia com: PuTTY / minicom na porta USB-CDC do Pico
    static stack_info_t infos[4];
    infos[0].h = h_mpu;    infos[0].name = "mpu_task";    infos[0].total_words = 8192;
    infos[1].h = h_fusion; infos[1].name = "fusion_task"; infos[1].total_words = 8192;
    infos[2].h = h_uart;   infos[2].name = "uart_task";   infos[2].total_words = 4096;
    infos[3].h = h_led;    infos[3].name = "led_task";    infos[3].total_words = 4096;

    xTaskCreate(stack_monitor_task, "StackMon", 1024, infos, 1, NULL);

    vTaskStartScheduler();

    while (true);
}