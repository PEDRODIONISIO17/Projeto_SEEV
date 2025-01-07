#include <Stepper.h>
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "HX711.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

// Configuração dos pinos
#define DT 16
#define SCK 4

#define TFT_CS   5
#define TFT_DC   26
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCLK 18
#define TFT_RST  -1 // ligar ao 3V3

// Configuração do motor
const int stepsPerRevolution = 2038; // Número de passos por rotação
Stepper myStepper1(stepsPerRevolution, 13, 14, 12, 27); // Pinos do motor

// Botões
const int buttonCW = 22;  // Botão para girar no sentido horário
const int buttonCCW = 21; // Botão para girar no sentido anti-horário

// LEDs
const int ledPin = 2; // Pino para o LED normal
const int ledRedPin = 25;  // Pino para o LED vermelho (excesso de peso)

// Criação do display
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);

// Semáforos
SemaphoreHandle_t displaySemaphore;
SemaphoreHandle_t motorStateMutex; 

// Filas
QueueHandle_t motorQueue;
QueueHandle_t scaleQueue;

// Variáveis globais
String motorState = "Parado";

// Comandos do motor
enum MotorCommand {
    MOTOR_STOP,
    MOTOR_CW,
    MOTOR_CCW
};

// Prototipação das tarefas e interrupções
void MotorTask(void *parameter);
void DisplayTask(void *parameter);
void LedTask(void *parameter);
void ScaleTask(void *parameter);
void IRAM_ATTR handleButtonCW();
void IRAM_ATTR handleButtonCCW();

void setup() {
    Serial.begin(115200);

    // Configuração dos botões como entrada com pull-up interno
    pinMode(buttonCW, INPUT_PULLUP);
    pinMode(buttonCCW, INPUT_PULLUP);

    motorStateMutex = xSemaphoreCreateMutex();

    // Configuração dos LEDs como saída
    pinMode(ledPin, OUTPUT);
    pinMode(ledRedPin, OUTPUT);  // LED vermelho

    // Configuração das interrupções
    attachInterrupt(digitalPinToInterrupt(buttonCW), handleButtonCW, FALLING);
    attachInterrupt(digitalPinToInterrupt(buttonCCW), handleButtonCCW, FALLING);

    // Criação das filas
    motorQueue = xQueueCreate(10, sizeof(MotorCommand));
    scaleQueue = xQueueCreate(10, sizeof(float));

    // Criação dos semáforos
    displaySemaphore = xSemaphoreCreateBinary();

    // Inicializar o semáforo como disponível
    xSemaphoreGive(displaySemaphore);

    // Criação das tarefas
    xTaskCreatePinnedToCore(MotorTask, "MotorTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 1000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(ScaleTask, "ScaleTask", 2048, NULL, 1, NULL, 0);

    // Configuração inicial do motor
    myStepper1.setSpeed(10); // Velocidade do motor (em RPM)

    // Inicializar o display
    xSemaphoreTake(displaySemaphore, portMAX_DELAY);
    tft.begin();
    tft.fillScreen(ILI9341_BLACK);
    tft.setRotation(0);

    // Desenhar elementos iniciais
    tft.setCursor(0, 10);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.println(" LINDE  ");
    tft.println(" INDUSTRIAS ");
    tft.println("     2024/2025      ");
    tft.drawRect(5, 5, 230, 90, ILI9341_RED);
    tft.drawRect(6, 6, 228, 88, ILI9341_RED); // Aumentar espessura
    tft.fillRoundRect(30, 105, 180, 28, 10, ILI9341_BLUE);
    tft.setCursor(55, 112);
    tft.println("DISPLAY LCD");

    // Exibir estado inicial do motor
    tft.setCursor(10, 150);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setTextSize(2);
    tft.println("Motor: Parado");

    xSemaphoreGive(displaySemaphore);
}

void loop() {
    // Nada aqui; o controle é feito pelas tarefas.
    vTaskDelete(NULL);
}

// Funções de interrupção para os botões
void IRAM_ATTR handleButtonCW() {
    MotorCommand cmd = MOTOR_CW;
    xQueueSendFromISR(motorQueue, &cmd, NULL);
}

void IRAM_ATTR handleButtonCCW() {
    MotorCommand cmd = MOTOR_CCW;
    xQueueSendFromISR(motorQueue, &cmd, NULL);
}

// Tarefa para controle do motor
void MotorTask(void *parameter) {
    MotorCommand cmd;

    while (true) {
        if (xQueueReceive(motorQueue, &cmd, portMAX_DELAY)) {
            switch (cmd) {
                case MOTOR_CW:
                    xSemaphoreTake(motorStateMutex, portMAX_DELAY);
                    motorState = "Horario";
                    xSemaphoreGive(motorStateMutex);
                    Serial.println("Gira no sentido horario");
                    while (digitalRead(buttonCW) == LOW) {
                        myStepper1.step(stepsPerRevolution / 100);
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    }
                    xSemaphoreTake(motorStateMutex, portMAX_DELAY);
                    motorState = "Parado";
                    xSemaphoreGive(motorStateMutex);
                    break;

                case MOTOR_CCW:
                    xSemaphoreTake(motorStateMutex, portMAX_DELAY);
                    motorState = "Anti-Horario";
                    xSemaphoreGive(motorStateMutex);
                    Serial.println("Gira no sentido anti-horario");
                    while (digitalRead(buttonCCW) == LOW) {
                        myStepper1.step(-stepsPerRevolution / 100);
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    }
                    xSemaphoreTake(motorStateMutex, portMAX_DELAY);
                    motorState = "Parado";
                    xSemaphoreGive(motorStateMutex);
                    break;

                case MOTOR_STOP:
                default:
                    xSemaphoreTake(motorStateMutex, portMAX_DELAY);
                    motorState = "Parado";
                    xSemaphoreGive(motorStateMutex);
                    break;
            }
        }
    }
}


// Tarefa para leitura da balança
void ScaleTask(void *parameter) {
    HX711 scale;
    scale.begin(DT, SCK);
    scale.set_scale();
    scale.tare(20);

    while (true) {
        float weight = scale.get_value(10);
        xQueueSend(scaleQueue, &weight, portMAX_DELAY);

        // Verificar se o peso excede 60.000g
        if (weight > 60000) {
            digitalWrite(ledRedPin, HIGH);  // Acende o LED vermelho
        } else {
            digitalWrite(ledRedPin, LOW);   // Apaga o LED vermelho
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

// Tarefa para atualizar o display
void DisplayTask(void *parameter) {
    String lastState = "";
    float lastWeight = 0;

    while (true) {
        String currentState;

        // Ler estado do motor com proteção
        xSemaphoreTake(motorStateMutex, portMAX_DELAY);
        currentState = motorState;
        xSemaphoreGive(motorStateMutex);

        if (currentState != lastState || uxQueueMessagesWaiting(scaleQueue) > 0) {
            // Atualizar estado do motor somente se mudou
            if (currentState != lastState) {
                lastState = currentState;

                if (xSemaphoreTake(displaySemaphore, portMAX_DELAY)) {
                    tft.fillRect(10, 150, 220, 30, ILI9341_BLACK); // Limpar área do estado do motor
                    tft.setCursor(10, 150);
                    tft.setTextColor(ILI9341_YELLOW);
                    tft.setTextSize(2);
                    tft.print("Motor: ");
                    tft.println(currentState);
                    xSemaphoreGive(displaySemaphore);
                }
            }

            // Atualizar peso
            if (xQueueReceive(scaleQueue, &lastWeight, 0)) {
                if (xSemaphoreTake(displaySemaphore, portMAX_DELAY)) {
                    tft.fillRect(10, 180, 220, 30, ILI9341_BLACK); // Limpar área do peso
                    tft.setCursor(10, 180);
                    tft.setTextColor(ILI9341_GREEN);
                    tft.setTextSize(2);
                    tft.print("Peso: ");
                    tft.print(lastWeight, 2);
                    tft.println(" g");
                    xSemaphoreGive(displaySemaphore);
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Verificar mudanças a cada 100 ms
    }
}

// Tarefa para controlar o LED
void LedTask(void *parameter) {
    String currentState;

    while (true) {
        xSemaphoreTake(motorStateMutex, portMAX_DELAY);
        currentState = motorState;
        xSemaphoreGive(motorStateMutex);

        if (currentState == "Horario" || currentState == "Anti-Horario") {
            digitalWrite(ledPin, HIGH);
            vTaskDelay(200 / portTICK_PERIOD_MS); // Piscar rápido
            digitalWrite(ledPin, LOW);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        } else if (currentState == "Parado") {
            digitalWrite(ledPin, HIGH);
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Piscar lento
            digitalWrite(ledPin, LOW);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
