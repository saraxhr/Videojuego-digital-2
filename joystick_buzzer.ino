#include <WiFi.h>
#include <esp_now.h>

const int joyX = 32;  // Pin del eje X del joystick local
const int joyY = 33;  // Pin del eje Y del joystick local
const int buzzerPin = 22;  // Pin del buzzer

const int movementThreshold = 500;  // Umbral para detectar movimiento significativo
const int debounceTime = 300;  // Tiempo de debounce en milisegundos
unsigned long lastMovementTime = 0;  // Para llevar la cuenta del debounce

int prevXValueLocal = 2048;  // Valor anterior del eje X (local)
int prevYValueLocal = 2048;  // Valor anterior del eje Y (local)
int prevXValueRemote = 2048;  // Valor anterior del eje X (remoto)
int prevYValueRemote = 2048;  // Valor anterior del eje Y (remoto)

// Estructura para recibir los datos del joystick remoto
typedef struct struct_message {
  int xValue;
  int yValue;
  int buttonState;
} struct_message;

struct_message incomingData;  // Datos recibidos del joystick remoto
struct_message joystickData;  // Datos para enviar del joystick local

// Dirección MAC del receptor central
uint8_t receptorMAC[] = {0xC8, 0xF0, 0x9E, 0xF5, 0x0D, 0x0C};

bool buzzerActive = false;  // Estado del buzzer

// Función para hacer sonar el buzzer
void beep() {
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
}

// Callback para recibir datos del joystick remoto (con firma corregida)
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));

  // Filtrar valores del joystick remoto
  int deltaXRemote = abs(incomingData.xValue - prevXValueRemote);
  int deltaYRemote = abs(incomingData.yValue - prevYValueRemote);

  // Verificar si el joystick remoto ha detectado movimiento significativo
  if ((deltaXRemote > movementThreshold || deltaYRemote > movementThreshold) && millis() - lastMovementTime > debounceTime) {
    Serial.println("Movimiento detectado en el joystick remoto, activando buzzer");
    buzzerActive = true;
    lastMovementTime = millis();  // Actualizar tiempo del último movimiento detectado
  }

  // Actualizar los valores previos del joystick remoto
  prevXValueRemote = incomingData.xValue;
  prevYValueRemote = incomingData.yValue;
}

// Callback para confirmar el envío de datos
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Paquete enviado a: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(mac_addr[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" con estado: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Error");
}

void setup() {
  Serial.begin(115200);
  
  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);
  pinMode(buzzerPin, OUTPUT);

  // Inicializar Wi-Fi en modo station
  WiFi.mode(WIFI_STA);

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al inicializar ESP-NOW");
    return;
  }

  // Registrar la función callback para recibir datos
  esp_now_register_recv_cb(OnDataRecv);

  // Registrar callback para enviar datos
  esp_now_register_send_cb(OnDataSent);

  // Añadir el receptor central
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receptorMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error al añadir el receptor central");
    return;
  }
}

// Función para detectar movimiento en el joystick local
bool detectLocalJoystickMovement(int xValue, int yValue) {
  int deltaXLocal = abs(xValue - prevXValueLocal);
  int deltaYLocal = abs(yValue - prevYValueLocal);

  // Verificar si hay movimiento significativo y aplicar debounce
  if ((deltaXLocal > movementThreshold || deltaYLocal > movementThreshold) && millis() - lastMovementTime > debounceTime) {
    lastMovementTime = millis();  // Actualizar tiempo del último movimiento
    return true;
  } else {
    return false;
  }
}

void loop() {
  // Leer valores del joystick local
  int xValueLocal = analogRead(joyX);
  int yValueLocal = analogRead(joyY);

  // Detectar movimiento en el joystick local
  if (detectLocalJoystickMovement(xValueLocal, yValueLocal)) {
    Serial.println("Movimiento detectado en el joystick local, enviando datos y activando buzzer");
    buzzerActive = true;

    // Enviar datos al receptor central
    joystickData.xValue = xValueLocal;
    joystickData.yValue = yValueLocal;
    esp_now_send(receptorMAC, (uint8_t *) &joystickData, sizeof(joystickData));
  }

  // Activar el buzzer si se detecta movimiento en el joystick local o remoto
  if (buzzerActive) {
    beep();
    buzzerActive = false;  // Apagar el buzzer después de sonar
  }

  // Actualizar los valores previos del joystick local
  prevXValueLocal = xValueLocal;
  prevYValueLocal = yValueLocal;

  delay(50);  // Reducir el tiempo de delay
}
