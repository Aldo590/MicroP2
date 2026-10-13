#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MOTOR_CONTROL";

// ==========================================
// CONFIGURACIÓN DE PINES
// ==========================================
#define PIN_PWM      GPIO_NUM_18
#define PIN_IN1      GPIO_NUM_19
#define PIN_IN2      GPIO_NUM_21
#define PIN_ENC_A    GPIO_NUM_26
#define PIN_ENC_B    GPIO_NUM_27

#define PWM_FREQ_HZ  20000
#define PWM_RES      LEDC_TIMER_10_BIT
#define PWM_MAX      1023

#define PULSOS_POR_REV_4X 937.0f
#define MAX_RPM           330.0f
#define TIEMPO_TOTAL_CICLO 10.0f
#define TIEMPO_PAUSA       2.0f
#define INTERVALO_PID_MS   20

// ==========================================
// MÁQUINA DE ESTADOS Y VARIABLES GLOBALES
// ==========================================
typedef enum {
    ESTADO_DETENIDO,
    ESTADO_ESPERANDO,
    ESTADO_CORRIENDO
} EstadoMotor_t;

volatile EstadoMotor_t estado_motor = ESTADO_DETENIDO;
volatile int64_t tiempoCambio_us = 0;

typedef enum {
    CONSTANTE, RAMPA_LINEAL, TRAPEZOIDAL, SENOIDAL,
    ESCALONADA, EXPONENCIAL, GAUSSIANO, DIENTE_SIERRA, RUIDO_BLANCO
} PerfilVelocidad_t;

volatile PerfilVelocidad_t perfilSeleccionado = TRAPEZOIDAL;
volatile PerfilVelocidad_t perfilActual = TRAPEZOIDAL;

// Variables PID
float Kp = 1.5f;
float Ki = 4.0f;
float Kd = 0.02f;
float errorIntegral = 0;
float rpmFiltrada = 0;
float rpmPrevia = 0;
const float ALPHA_FILTRO = 0.3f;

pcnt_unit_handle_t pcnt_unit = NULL;

// ==========================================
// PERFILES MATEMÁTICOS
// ==========================================
float evaluarPerfil(PerfilVelocidad_t perfil, float t, float t_final) {
    float p0 = 0.0f;
    float p_max = 280.0f;

    switch (perfil) {
        case CONSTANTE:     return 200.0f;
        case RAMPA_LINEAL:  return p0 + ((p_max - p0) / t_final) * t;
        case TRAPEZOIDAL: {
            float ta = t_final * 0.3f, tb = t_final * 0.7f;
            float a = p_max / ta;
            if (t < ta) return a * t;
            if (t < tb) return p_max;
            return p_max - a * (t - tb);
        }
        case SENOIDAL:      return 150.0f + 100.0f * sinf(2.0f * M_PI * 0.5f * t);
        case ESCALONADA:    return (fmodf(t, 2.0f) < 1.0f) ? 50.0f : 250.0f;
        case EXPONENCIAL:   return p_max * powf(t / t_final, 2.0f);
        case GAUSSIANO:     return p_max * expf(-powf((t - (t_final/2.0f)) / (t_final/5.0f), 2.0f));
        case DIENTE_SIERRA: return p_max * fmodf(t / 2.5f, 1.0f);
        case RUIDO_BLANCO:  return 180.0f;
        default:            return 0.0f;
    }
}

// ==========================================
// HARDWARE Y PWM
// ==========================================
void init_hardware() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_IN1) | (1ULL << PIN_IN2);
    gpio_config(&io_conf);
    
    gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 0);

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution = PWM_RES;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.freq_hz = PWM_FREQ_HZ;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num = PIN_PWM;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&ledc_channel);

    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = -32000; unit_config.high_limit = 32000;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1000;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = PIN_ENC_A; chan_a_config.level_gpio_num = PIN_ENC_B;
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num = PIN_ENC_B; chan_b_config.level_gpio_num = PIN_ENC_A;
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void aplicarMotorPWM(int pwm_value) {
    if (pwm_value > 0) {
        gpio_set_level(PIN_IN1, 1); gpio_set_level(PIN_IN2, 0);
    } else if (pwm_value < 0) {
        gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 1);
    } else {
        gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 0);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwm_value));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ==========================================
// TAREA 1: CONTROL PID (Maneja el Motor)
// ==========================================
void control_motor_task(void *pvParameter) {
    int64_t tiempoInicioCiclo_us = esp_timer_get_time();
    int64_t tiempoUltimoPID_us = esp_timer_get_time();

    while (1) {
        int64_t tiempoActual_us = esp_timer_get_time();
        float dt = (tiempoActual_us - tiempoUltimoPID_us) / 1000000.0f;
        
        if (dt >= (INTERVALO_PID_MS / 1000.0f)) {
            tiempoUltimoPID_us = tiempoActual_us;
            float objetivo = 0.0f;

            // === LÓGICA DE ESTADOS ===
            if (estado_motor == ESTADO_DETENIDO) {
                objetivo = 0.0f;
                tiempoInicioCiclo_us = tiempoActual_us; // Mantener el timer de ciclo a 0
            
            } else if (estado_motor == ESTADO_ESPERANDO) {
                objetivo = 0.0f;
                // Si ya pasaron 3 segundos (3,000,000 microsegundos)
                if ((tiempoActual_us - tiempoCambio_us) >= 3000000LL) {
                    estado_motor = ESTADO_CORRIENDO;
                    perfilActual = perfilSeleccionado;
                    tiempoInicioCiclo_us = tiempoActual_us; // Comienza el ciclo en t=0
                    printf(">Info: ¡Arrancando motor!\n");
                }
            
            } else if (estado_motor == ESTADO_CORRIENDO) {
                float t_ciclo = (tiempoActual_us - tiempoInicioCiclo_us) / 1000000.0f;
                if (t_ciclo <= TIEMPO_TOTAL_CICLO) {
                    objetivo = evaluarPerfil(perfilActual, t_ciclo, TIEMPO_TOTAL_CICLO);
                } else if (t_ciclo <= (TIEMPO_TOTAL_CICLO + TIEMPO_PAUSA)) {
                    objetivo = 0.0f;
                } else {
                    tiempoInicioCiclo_us = esp_timer_get_time(); // Reiniciar ciclo
                }
            }

            // === LECTURA DE ENCODER Y FILTRO ===
            int pulsos = 0;
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulsos));
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

            float rpmCruda = ((float)pulsos / PULSOS_POR_REV_4X) * (60.0f / dt);
            rpmFiltrada = (ALPHA_FILTRO * rpmCruda) + ((1.0f - ALPHA_FILTRO) * rpmFiltrada);

            // === CONTROL PID ===
            float error = objetivo - fabsf(rpmFiltrada);
            float P = Kp * error;
            float derivadaMedicion = (fabsf(rpmFiltrada) - fabsf(rpmPrevia)) / dt;
            float D = -Kd * derivadaMedicion;
            rpmPrevia = rpmFiltrada;

            float salidaControl = P + (Ki * errorIntegral) + D;

            if (salidaControl > PWM_MAX) {
                salidaControl = PWM_MAX;
                if (error < 0) errorIntegral += error * dt;
            } else if (salidaControl < -PWM_MAX) {
                salidaControl = -PWM_MAX;
                if (error > 0) errorIntegral += error * dt;
            } else {
                errorIntegral += error * dt;
            }

            // Parada total limpia sin acumular integral
            if (objetivo == 0.0f && fabsf(rpmFiltrada) < 2.0f) {
                salidaControl = 0;
                errorIntegral = 0;
            }

            aplicarMotorPWM((int)salidaControl);
            
            // Envío a Teleplot (solo grafica si está corriendo, si no manda 0 para gráfica limpia)
            printf(">Objetivo:%.2f\n", objetivo);
            printf(">RPM_Real:%.2f\n", fabsf(rpmFiltrada));
            printf(">PWM:%.2f\n", salidaControl);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================
// TAREA 2: LECTURA SERIAL (Maneja el Menú)
// ==========================================
void lectura_serial_task(void *pvParameter) {
    char linea[32];
    
    // Imprimir el menú al arrancar (se muestra en la terminal de Teleplot)
    printf("\n>Info: === CONTROL DE MOTOR LISTO ===\n");
    printf(">Info: Comandos disponibles:\n");
    printf(">Info: [1-9] Selecciona perfil y arranca en 3s\n");
    printf(">Info: [s] o [0] Detener de emergencia\n");
    printf(">Info: Estado inicial: DETENIDO.\n\n");

    while (1) {
        // Se queda bloqueado aquí esperando a que escribas y presiones Enter
        if (fgets(linea, sizeof(linea), stdin) != NULL) {
            char comando = linea[0];
            
            if (comando >= '1' && comando <= '9') {
                perfilSeleccionado = (PerfilVelocidad_t)(comando - '1');
                tiempoCambio_us = esp_timer_get_time();
                estado_motor = ESTADO_ESPERANDO;
                printf(">Info: Curva %c solicitada. Arrancando en 3 segundos...\n", comando);
                
            } else if (comando == 's' || comando == 'S' || comando == '0') {
                estado_motor = ESTADO_DETENIDO;
                printf(">Info: --- MOTOR DETENIDO DE EMERGENCIA ---\n");
            }
        }
        // Pequeño descanso para el sistema
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==========================================
// FUNCIÓN PRINCIPAL
// ==========================================
extern "C" void app_main(void) {
    init_hardware();
    
    // Tarea del motor (Core 1, Alta prioridad = 5)
    xTaskCreatePinnedToCore(control_motor_task, "motor_task", 4096, NULL, 5, NULL, 1);
    
    // Tarea de lectura serial (Core 0, Baja prioridad = 2)
    xTaskCreatePinnedToCore(lectura_serial_task, "serial_task", 3072, NULL, 2, NULL, 0);
}