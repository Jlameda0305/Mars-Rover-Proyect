# Feature: Robotic Arm Standalone Controller & Test Suite

**Branch:** `feature/robotic-arm`  
**Fecha de inicio:** 2026-10-14  
**Estado:** En Progreso  
**Ruta de Implementación:** Delegada Directa (Writer Trigger: 2+ archivos no triviales)  
**TDD Mode:** Desactivado (TDD off por defecto de configuración; pruebas funcionales unitarias nativas dedicadas con g++)  
**Test Runner:** `g++ -std=c++17 test/test_arm_kinematics.cpp src/arm_kinematics.cpp -Isrc -o test/run_tests && ./test/run_tests`  
**Estrategia de Entrega:** `ask-on-risk` (Pronóstico: ~450 líneas de adición/modificación)  
**Espejo Engram:** `odd/robotic-arm/tasks` (Pendiente: servicio Engram no disponible en entorno actual)

---

## 1. Objetivo y Justificación
Extraer y aislar exclusivamente las funcionalidades asociadas al **brazo robótico de 6 grados de libertad (6-DOF)** del proyecto Mars Rover, eliminando todos los subsistemas no relacionados (chasis 6x6, dirección en las 4 ruedas, mástil de cámara pan/tilt, sensor IMU MPU-6050, transceptor LoRa y receptor RMT).

Garantizar que el sistema sea:
1. **Completamente funcional:** Operable de extremo a extremo desde `main.cpp` (firmware ESP32 + WiFi AP) hasta el panel de control web responsivo en `data/index.html`.
2. **Testeable:** Con lógica cinemática y de interpolación desacoplada del hardware (`IPwmDriver`), permitiendo la compilación y ejecución directa de pruebas unitarias en Linux con `g++`.

---

## 2. Alcance Autorizado y Restricciones
- **Autorizado:**
  - Creación y trabajo en la rama `feature/robotic-arm`.
  - Extracción y refactorización de la lógica del brazo robótico (Ejes 1 a 6: Base, Hombro, Codo, Muñeca Pitch, Muñeca Roll, Pinza).
  - Creación de interfaces desacopladas de cinemática y actuadores (`arm_kinematics.h/.cpp`, `arm_config.h`, `IPwmDriver`).
  - Creación de suite de pruebas unitarias nativas en `test/` ejecutable con `g++`.
  - Depuración y eliminación de archivos de tracción, dirección, cámara, LoRa, RMT y switch multipropósito.
  - Rediseño integral de `data/index.html` para control dedicado de los 6 ejes, visualización de ángulos/pulsos, presets, jogging y parada de emergencia.
  - Actualización de `main.cpp`, `arm.cpp`, `arm_servo.cpp` y `wifi_controller.cpp`.
- **Restricciones:**
  - Preservar la compatibilidad física de los servos en el PCA9685 (Canales 0..5, I2C pines 21/22, dirección 0x40).
  - No romper la rama `main`.
  - Toda operación remota o git push requiere autorización expresa (operación puramente local).

---

## 3. Lista de Tareas (Checklist)

- [x] **TASK-01:** Refactorización y Aislamiento del Núcleo Cinemático y de Control del Brazo (`arm_config.h`, `arm_kinematics.h`, `arm_kinematics.cpp`, `arm_driver.h`)
  - *Ruta:* Delegada (Writer Trigger)
  - *Evidencia observada:* Clases y funciones cinemáticas implementadas sin dependencias de FreeRTOS ni Arduino. Compilación exitosa en `g++ -std=c++17 -Wall -Wextra -Werror`.
  - *Commit:* `416dfb5`
  - *RDD Assess:* `unavailable` (runtime 'antigravity' no elegible para revisión inmutable por gentle-ai).

- [x] **TASK-02:** Implementación de la Suite de Pruebas Unitarias Nativas (`test/test_arm_kinematics.cpp`)
  - *Ruta:* Delegada (Writer Trigger)
  - *Evidencia observada:* Suite nativa ejecutable con `g++ -std=c++17` validando 8 suites y 891 aserciones (nombres/límites de articulaciones, clamping de pulso y ángulo, ida y vuelta ángulo-pulso, cuentas PCA9685 para 1000/1500/2000/600/2400 us, cálculo de pasos de trayectoria suave con escalado de velocidad y sin sobrepaso, validez de presets HOME y REST, y MockPwmDriver implementando IPwmDriver). 100% de aserciones exitosas.
  - *Commit:* `70b5230`
  - *RDD Assess:* `unavailable` (runtime 'antigravity').

- [x] **TASK-03:** Adaptación de la Capa de Hardware Embebida (`arm_servo.cpp`, `arm.cpp`, `main.cpp`) y Eliminación de Módulos No Relacionados
  - *Ruta:* Delegada (Writer Trigger)
  - *Evidencia observada:* Eliminación de los 16 archivos de subsistemas ajenos al brazo (`rover_driving`, `rover_head`, `gyro_accel_sensor`, `rc_receiver_rmt`, `lora_controller`, `rover_settings_switch`, `switch_checker`, `rover_servo`). Implementación de `arm_servo.h/.cpp` con control thread-safe FreeRTOS del driver PCA9685 y tarea de fondo `ArmServoUpdater` a 10ms con `calculate_step`. Refactorización de `arm.h/.cpp` con API de alto nivel y compatibilidad previa. `main.cpp` limpio arrancando directamente el brazo a HOME y WiFi AP "Robotic-Arm" (192.168.4.5) con parada de emergencia ante desconexión. `platformio.ini` depurado con solo dependencias de PCA9685 y AsyncWebServer. Suite de pruebas nativas pasando 100%.
  - *Commit:* `4a4b58b`
  - *RDD Assess:* `unavailable` (runtime 'antigravity').

- [ ] **TASK-04:** Protocolo de Comunicación y Dashboard Web Dedicado (`wifi_controller.cpp`, `data/index.html`)
  - *Ruta:* Delegada (Writer Trigger)
  - *Evidencia esperada:* `index.html` con HUD responsivo para los 6 ejes (Base, Hombro, Codo, Muñeca Pitch, Muñeca Roll, Pinza), sliders de grado/pulso, jogging táctil, presets (Home/Rest/Open/Close), y WebSocket bidireccional.
  - *Commit:* Conventional commit en rama `feature/robotic-arm`.
  - *RDD Assess:* Pendiente de commit.

- [ ] **TASK-05:** Verificación Funcional Integral End-to-End, Ejecución de Pruebas y Cierre
  - *Ruta:* Directa inline
  - *Evidencia esperada:* Ejecución completa de pruebas nativas, verificación estática de archivos y reporte final.
  - *Commit:* Conventional commit en rama `feature/robotic-arm`.
  - *RDD Assess:* Pendiente de commit.

---

## 4. Registro de Verificación y Evidencias
- *TASK-01 Commit:* `416dfb5` (feat(arm): implement decoupled kinematics and configuration layer (TASK-01))
- *TASK-02 Commit:* `70b5230` (test(arm): implement native test suite and runner for kinematics and mock driver (TASK-02))
- *TASK-03 Commit:* `4a4b58b` (refactor(arm): decouple embedded hardware layer and remove non-arm subsystems (TASK-03))
- *TASK-04 Commit:* Pendiente
- *TASK-05 Commit:* Pendiente

