#include <WiFi.h>
#include <esp_now.h>

// Estructura para almacenar los datos del joystick
typedef struct struct_message {
  int xValue;
  int yValue;
  int buttonState;
} struct_message;

struct_message joystickData1;  // Joystick remoto 1
struct_message joystickData2;  // Joystick remoto 2

// Callback para recibir los datos de cualquier joystick
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  // Suponiendo que ambos joysticks envían la misma estructura de datos
  if (len == sizeof(joystickData1)) {
    memcpy(&joystickData1, data, sizeof(joystickData1));
    Serial.println("Datos recibidos de un joystick:");
    Serial.print("X: ");
    Serial.println(joystickData1.xValue);
    Serial.print("Y: ");
    Serial.println(joystickData1.yValue);
    Serial.print("Button: ");
    Serial.println(joystickData1.buttonState);
  } else {
    Serial.println("Tamaño de paquete incorrecto");
  }
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

  // Registrar la función callback para recibir datos
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Receptor listo para recibir datos.");
}

void loop() {
  // El receptor se mantiene a la espera de los datos
  delay(100);
}

