 * ============================================================
 * Proyecto: Robot Minisumo
 * Autor:    Jorge Guillermo Puente León - 24150
 * Curso:    Simulación de Circuitos y Fabricación de PCB
 * Fecha:    mayo 2026
 * Version:  1.0.0
 * 
 * Descripción:
 *   Firmware para control inalámbrico de robot Minisumo
 *   mediante DualShock 4 y Bluetooth usando Bluepad32.
 *   El joystick izquierdo controla el Motor A y el joystick
 *   derecho controla el Motor B (configuración tipo tank).
 * 
 * Dependencias:
 *   - Bluepad32 (incluida en el board package ESP32+Bluepad32)
 * 
 * Historial de versiones:
 *   v1.0.0 - Mayo 2026 - Versión inicial funcional
 * ============================================================
 */

#include <Bluepad32.h>

// ——————————————————————————————————————————
// DEFINICIÓN DE PINES
// ——————————————————————————————————————————

// Motor A — conectado a J6 del PCB
#define AIN1  27    // Dirección Motor A - pin 3 de J6
#define AIN2  26    // Dirección Motor A - pin 2 de J6
#define PWMA  25    // Velocidad Motor A (PWM) - pin 1 de J6

// Motor B — conectado a J7 del PCB
#define BIN1  18    // Dirección Motor B - pin 3 de J7
#define BIN2  33    // Dirección Motor B - pin 2 de J7
#define PWMB  32    // Velocidad Motor B (PWM) - pin 1 de J7

// Standby del TB6612FNG — conectado a J9 del PCB
// HIGH = driver activo | LOW = driver en standby (motores apagados)
#define STBY  23

// ——————————————————————————————————————————
// CONFIGURACIÓN PWM (periférico LEDC del ESP32)
// ——————————————————————————————————————————
#define CH_A      0     // Canal LEDC para Motor A
#define CH_B      1     // Canal LEDC para Motor B
#define PWM_FREQ  1000  // Frecuencia PWM en Hz
#define PWM_RES   8     // Resolución: 8 bits = valores de 0 a 255

// ——————————————————————————————————————————
// ZONA MUERTA DEL JOYSTICK
// Evita que ruido en reposo del joystick mueva los motores
// Rango del eje: -511 a 512. Valores menores a DEADZONE se ignoran.
// ——————————————————————————————————————————
#define DEADZONE 30

// Array de controladores — Bluepad32 soporta hasta 4 simultáneos
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ——————————————————————————————————————————
// FUNCIÓN: setMotorA
// Controla velocidad y dirección del Motor A
// Parámetro speed: -255 (atrás máx) a 255 (adelante máx), 0 = freno
// ——————————————————————————————————————————
void setMotorA(int speed) {
  if (speed > 0) {
    // Adelante: AIN1=HIGH, AIN2=LOW
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(CH_A, speed);
  } else if (speed < 0) {
    // Atrás: AIN1=LOW, AIN2=HIGH
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(CH_A, -speed); // ledcWrite solo acepta positivos
  } else {
    // Freno: ambos LOW, PWM = 0
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(CH_A, 0);
  }
}

// ——————————————————————————————————————————
// FUNCIÓN: setMotorB
// Controla velocidad y dirección del Motor B
// Parámetro speed: -255 (atrás máx) a 255 (adelante máx), 0 = freno
// ——————————————————————————————————————————
void setMotorB(int speed) {
  if (speed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(CH_B, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(CH_B, -speed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(CH_B, 0);
  }
}

// ——————————————————————————————————————————
// CALLBACK: onConnectedController
// Se ejecuta automáticamente cuando el DualShock 4
// establece conexión Bluetooth con el ESP32
// ——————————————————————————————————————————
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("Control conectado, index=%d\n", i);
      myControllers[i] = ctl;
      digitalWrite(STBY, HIGH); // Activa el TB6612FNG al conectar
      break;
    }
  }
}

// ——————————————————————————————————————————
// CALLBACK: onDisconnectedController
// Se ejecuta automáticamente cuando el control
// pierde la conexión Bluetooth
// ——————————————————————————————————————————
void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("Control desconectado, index=%d\n", i);
      myControllers[i] = nullptr;
      setMotorA(0);          // Freno de seguridad al desconectar
      setMotorB(0);
      digitalWrite(STBY, LOW); // Desactiva el TB6612FNG
      break;
    }
  }
}

// ——————————————————————————————————————————
// FUNCIÓN: processGamepad
// Procesa los datos del joystick y los convierte
// en velocidades para cada motor
// ——————————————————————————————————————————
void processGamepad(ControllerPtr ctl) {

  // Leer ejes Y de ambos joysticks
  // Rango nativo Bluepad32: -511 a 512
  // Negativo = arriba (adelante), Positivo = abajo (atrás)
  int ly = ctl->axisY();   // Joystick izquierdo → Motor A
  int ry = ctl->axisRY();  // Joystick derecho   → Motor B

  // Aplicar zona muerta: ignorar valores pequeños por ruido
  if (abs(ly) < DEADZONE) ly = 0;
  if (abs(ry) < DEADZONE) ry = 0;

  // Convertir rango -511~512 a -255~255 para el PWM
  // Se invierte el signo porque en el DS4 arriba = negativo
  int speedA = -map(ly, -511, 512, -255, 255);
  int speedB = -map(ry, -511, 512, -255, 255);

  // Enviar velocidades calculadas a cada motor
  setMotorA(speedA);
  setMotorB(speedB);

  // Debug: mostrar valores en Serial Monitor (115200 baud)
  Serial.printf("Motor A: %4d | Motor B: %4d\n", speedA, speedB);
}

// ——————————————————————————————————————————
// FUNCIÓN: processControllers
// Itera sobre todos los controladores conectados
// y procesa solo los que son gamepads con datos nuevos
// ——————————————————————————————————————————
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
    }
  }
}

// ——————————————————————————————————————————
// SETUP — Se ejecuta una vez al encender el ESP32
// ——————————————————————————————————————————
void setup() {
  Serial.begin(115200);
  Serial.println("AlfaJOR v1.0.0 iniciando...");

  // Configurar pines de dirección como salidas
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // TB6612 en standby hasta que el control se conecte
  digitalWrite(STBY, LOW);

  // Inicializar canales PWM con LEDC
  ledcSetup(CH_A, PWM_FREQ, PWM_RES); // Canal 0: Motor A
  ledcSetup(CH_B, PWM_FREQ, PWM_RES); // Canal 1: Motor B
  ledcAttachPin(PWMA, CH_A);
  ledcAttachPin(PWMB, CH_B);

  // Inicializar Bluepad32 con los callbacks de conexión
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Limpiar emparejamientos previos para evitar reconexiones no deseadas
  BP32.forgetBluetoothKeys();

  // Deshabilitar dispositivo virtual (touchpad del DS4 como mouse)
  BP32.enableVirtualDevice(false);

  Serial.println("Esperando DualShock 4... (PS + Share para emparejar)");
}

// ——————————————————————————————————————————
// LOOP — Se ejecuta continuamente
// ——————————————————————————————————————————
void loop() {
  // Actualizar datos de todos los controladores conectados
  bool dataUpdated = BP32.update();

  // Solo procesar si hay datos nuevos del control
  if (dataUpdated)
    processControllers();

  // Delay necesario para no saturar el watchdog del ESP32
  delay(20);
}
