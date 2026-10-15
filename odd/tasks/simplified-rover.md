# Feature: Simplified Rover (MVP Conducción RC)

**Branch:** `feature/simplified-rover`  
**Fecha de inicio:** 2026-10-14  
**Estado:** En Progreso  
**Ruta de Implementación:** Delegada Directa (Writer Trigger: 2+ archivos no triviales)  
**TDD Mode:** Desactivado (Verificación funcional en compilación/firmware embebido)  
**Estrategia de Entrega:** `ask-on-risk` (Presupuesto estimado de cambios: ~150-200 líneas)  
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
- [x] **TASK-03:** Verificación de sintaxis, integridad del sistema y documentación de prueba.
  - *Ruta:* Directa inline
  - *Evidencia observada:* Verificación de dependencias y sintaxis OK, documentación de prueba y pasos de flasheo completados.
  - *Commit:* Pendiente (próximo commit de cierre)

---

## 4. Registro de Verificación y Evidencias
- *TASK-01 Commit:* `9e53a32`
- *TASK-02 Commit:* `0cb1eed`
- *TASK-03 Commit:* Pendiente
- *Running Changed Lines:* ~450 líneas authored (additions + deletions excluyendo autogenerados)

