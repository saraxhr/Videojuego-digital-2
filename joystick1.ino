#include <WiFi.h>
#include <esp_now.h>

const int joyX = 32;  // Pin del eje X (GPIO32)
const int joyY = 33;  // Pin del eje Y (GPIO33)
const int movementThreshold = 1000;  // Umbral para detectar movimiento

int prevXValue = 2048;
int prevYValue = 2048;

// Estructura para enviar los datos del joystick
typedef struct struct_message {
  int xValue;
  int yValue;
  int buttonState;
} struct_message;

struct_message outgoingData;

// Dirección MAC del ESP32 receptor con buzzer y del ESP32 receptor central
uint8_t addressReceiver[] = {0x30, 0xAE, 0xA4, 0xEB, 0x23, 0x50};  // Dirección MAC del ESP32 con buzzer
uint8_t addressCentralReceiver[] = {0xC8, 0xF0, 0x9E, 0xF5, 0x0D, 0x0C};  // Dirección MAC del receptor central

// Callback para saber si el envío fue exitoso
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Datos enviados con éxito" : "Error al enviar datos");
}

void setup() {
  Serial.begin(115200);

  // Inicializar WiFi en modo Station
  WiFi.mode(WIFI_STA);

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al inicializar ESP-NOW");
    return;
  }

  // Registrar la función callback para saber si los datos fueron enviados
  esp_now_register_send_cb(OnDataSent);

  // Añadir el peer del ESP32 con buzzer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, addressReceiver, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error al añadir el peer con buzzer");
    return;
  }

  // Añadir el peer del ESP32 receptor central
  memcpy(peerInfo.peer_addr, addressCentralReceiver, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error al añadir el receptor central");
    return;
  }
}

void loop() {
  // Obtener los valores del joystick
  outgoingData.xValue = analogRead(joyX);
  outgoingData.yValue = analogRead(joyY);

  // Verificar si hay movimiento significativo en el joystick
  if (abs(outgoingData.xValue - prevXValue) > movementThreshold || abs(outgoingData.yValue - prevYValue) > movementThreshold) {
    // Enviar datos al ESP32 con buzzer
    esp_now_send(addressReceiver, (uint8_t *) &outgoingData, sizeof(outgoingData));

    // Enviar datos al ESP32 receptor central
    esp_now_send(addressCentralReceiver, (uint8_t *) &outgoingData, sizeof(outgoingData));
  }

  // Actualizar valores previos
  prevXValue = outgoingData.xValue;
  prevYValue = outgoingData.yValue;

  delay(20);  // Aumentar la frecuencia de actualización
}

