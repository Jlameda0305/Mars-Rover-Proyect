# Feature: Simplified Rover (MVP Conducción RC)

**Branch:** `feature/simplified-rover`  
**Fecha de inicio:** 2026-10-14  
**Estado:** Completado  
**Ruta de Implementación:** Delegada Directa (Writer Trigger: 2+ archivos no triviales)  
**TDD Mode:** Desactivado (TDD off; pruebas funcionales unitarias nativas dedicadas con g++)  
**Test Runner:** `g++ -std=c++17 test/test_rover_kinematics.cpp src/rover_kinematics.cpp -Isrc -o test/run_tests && ./test/run_tests`  
**Estrategia de Entrega:** `ask-on-risk` (Pronóstico: ~800 líneas)  
**Espejo Engram:** `odd/simplified-rover/tasks` (Pendiente / Memoria local activa)


---

## 1. Objetivo y Justificación
Desarrollar una versión simplificada del rover enfocada exclusivamente en la motricidad (conducción tipo auto a control remoto) para pruebas iniciales en banco y terreno.
Se desacoplan el brazo robótico (6 DOF), el mástil de la cámara (2 DOF), el transceptor de radio LoRa y el decodificador de radiofrecuencia RMT físico, permitiendo que el ESP32 levante de forma autónoma una red WiFi en modo Access Point ("Rover") y sea controlado directamente a través del portal web en `index.html`.

---

## 2. Alcance Autorizado y Restricciones
- **Autorizado:**
  - Creación de la rama `feature/simplified-rover`.
  - Refactorización de `src/main.cpp` para forzar modo WiFi AP, omitir inicializaciones de brazo/cabeza/LoRa/RMT y conectar directamente los comandos WebSockets a `rover_driving`.
  - Simplificación de `data/index.html` para eliminar botones y elementos huérfanos (Arm, Gripper, Head, Camera, LED) y optimizar los joysticks virtuales para aceleración y dirección.
  - Verificación estática y de integridad del código.
- **Restricciones:**
  - Preservar la lógica matemática de dirección 4WS y tracción diferencial en `rover_driving.cpp`.
  - Preservar la compatibilidad de pines de hardware en `rover_config.h`.
  - No afectar la rama principal `main`.

---

## 3. Lista de Tareas (Checklist)

- [x] **TASK-01:** Simplificar `src/main.cpp` para control directo vía WiFi AP sin dependencias de LoRa, RMT, brazo ni mástil.
  - *Ruta:* Delegada (Writer Trigger)
  - *Evidencia observada:* `src/main.cpp` refactorizado a 123 líneas, inicializa directamente WiFi AP y puentea canales hacia `rover_driving`. `wifi_controller.h` incluye alias de compatibilidad.
  - *Commit:* `9e53a32` (`feat(firmware): simplify main to standalone wifi ap rc mode`)
  - *RDD Assessment:* Inaccesible / No disponible (runtime antigravity no admitido por ejecutor inmutable de gentle-ai).
- [x] **TASK-02:** Rediseñar y simplificar `data/index.html` para interfaz dedicada de auto RC.
  - *Ruta:* Delegada (Writer Trigger)
  - *Evidencia observada:* UI responsive moderna con joysticks duales para Acelerador/Dirección, modos Normal (4WS) y Spin (360°), botón EMERGENCY STOP, telemetría e indicador dinámico de conexión. Validación sintáctica OK.
  - *Commit:* `0cb1eed` (`feat(web): redesign index.html for dedicated rc car driving interface`)
  - *RDD Assessment:* Inaccesible / No disponible.
- [x] **TASK-03:** Verificación inicial de sintaxis, integridad del sistema y documentación de prueba.
  - *Ruta:* Directa inline
  - *Evidencia observada:* Verificación de dependencias y sintaxis OK, documentación de prueba y pasos de flasheo completados.
  - *Commit:* `1301746` (`docs: record task completion and verification for simplified rover`)
- [x] **TASK-04:** Depuración completa de subsistemas ajenos (`arm`, `rover_head`, `lora`, `rc_receiver_rmt`, `rover_settings_switch`, `gyro_accel_sensor`, `switch_checker`) y limpieza de `platformio.ini`.
  - *Ruta:* Delegada (Writer Trigger)
  - *Evidencia observada:* Eliminación de 14 archivos huérfanos en `src/`, migración de `RoverMode` a `rover_driving.h`, remoción de `switch_checker.h` de `main.cpp` y purga de dependencias en `platformio.ini`.
  - *Commit:* `b79012f` (`refactor(firmware): purge orphan subsystems and clean platformio dependencies`)
- [x] **TASK-05:** Desacoplamiento de la cinemática de motricidad (`rover_kinematics.h/.cpp`) y creación de la suite de pruebas nativas (`test/test_rover_kinematics.cpp`).
  - *Ruta:* Delegada (Writer Trigger)
  - *Evidencia observada:* Módulo puro `rover_kinematics.h/.cpp` implementado con cero dependencias Arduino/FreeRTOS. Integrado en `rover_driving.cpp`, `main.cpp` y `rover_servo.cpp`. Suite nativa `test_rover_kinematics.cpp` ejecutada con `g++ -std=c++17` superando 6 suites y 184/184 aserciones (100%).
  - *Commit:* `190ec4a` (`feat(kinematics): decouple kinematic calculations and add native unit test suite`)
- [x] **TASK-07:** Migración a control directo GPIO de los 4 servos de dirección (eliminación de PCA9685 e I2C).
  - *Ruta:* Delegada (Writer Trigger: 2+ archivos no triviales)
  - *Alcance:*
    - Actualizar `src/rover_config.h` con pines GPIO 18, 19, 21, 22 para los servos de dirección y remover definiciones de I2C.
    - Modificar `src/rover_servo.h` y `src/rover_servo.cpp` para instanciar y controlar los 4 servos directamente con `ESP32Servo`, eliminando `Adafruit_PWMServoDriver`, `Wire.h` y semáforos I2C.
    - Actualizar `src/main.cpp` para remover `Wire.begin` y la creación del semáforo I2C.
    - Actualizar `platformio.ini` para remover la dependencia `Adafruit PWM Servo Driver Library`.
    - Actualizar `diagram.json` para reflejar los 6 servos directos en Wokwi.
    - Actualizar `libraries.txt` y `sketch.ino`.
    - Ejecutar pruebas nativas y pruebas de compilación PlatformIO.
  - *Evidencia observada:* Compilación PlatformIO exitosa ([SUCCESS], Flash: 69.3%, RAM: 14.6%). Suite nativa test_rover_kinematics.cpp aprobada al 100% (6/6 suites, 184/184 aserciones).
  - *Commit:* `82450fc` (`feat(servo): migrate steering servos to direct GPIO control with ESP32Servo`)

---

## 4. Registro de Verificación y Evidencias
- *TASK-01 Commit:* `9e53a32` (feat(firmware): simplify main to standalone wifi ap rc mode)
- *TASK-02 Commit:* `0cb1eed` (feat(web): redesign index.html for dedicated rc car driving interface)
- *TASK-03 Commit:* `1301746` (docs: record task completion and verification for simplified rover)
- *TASK-04 Commit:* `b79012f` (refactor(firmware): purge orphan subsystems and clean platformio dependencies)
- *TASK-05 Commit:* `190ec4a` (feat(kinematics): decouple kinematic calculations and add native unit test suite)
- *TASK-06 Commit:* `e21b264` (docs(odd): complete TASK-06 and close simplified-rover feature)
- *TASK-07 Commit:* `82450fc` (feat(servo): migrate steering servos to direct GPIO control with ESP32Servo)
- *Running Changed Lines:* ~1150 líneas authored (additions + deletions excluyendo autogenerados)



