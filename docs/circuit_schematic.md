# Esquemas de Ingeniería y Diagramas de Conexión: Mars Rover (Direct GPIO)

Este documento reúne el conjunto completo de planos esquemáticos técnicos de ingeniería para el Mars Rover, diseñados bajo estándar CAD/norma técnica con cuadrícula de coordenadas, código de colores ortogonales y bloques de rotulación.

---

## 1. Esquema 1: Conexión y Control de Señales (ESP32 Direct GPIO)

Control directo de los 6 actuadores (2 ESCs de tracción y 4 servomotores MG996R de dirección) sin módulo PCA9685, empleando los temporizadores de hardware PWM (LEDC) del ESP32.

![Diagrama General del Circuito](/home/jlameda/.gemini/antigravity/brain/e3db32e4-bfca-4eef-84a5-4d985584dddd/schematic_circuit_diagram.png)

- **Archivos fuente:** [`docs/schematic_circuit_diagram.svg`](file:///home/jlameda/Documents/Mars-Rover-Proyect/docs/schematic_circuit_diagram.svg) | [PNG](file:///home/jlameda/Documents/Mars-Rover-Proyect/docs/schematic_circuit_diagram.png) | [PDF](file:///home/jlameda/Documents/Mars-Rover-Proyect/docs/schematic_circuit_diagram.pdf)

### Mapeo de Señales (Pinout)
| Componente | Designador | Pin ESP32 | Color de Línea | Función Técnica |
| :--- | :---: | :---: | :---: | :--- |
| **ESC Motor Izquierdo** | **M1** | **GPIO 17** | Azul (`#2980B9`) | Control PWM Tracción Izquierda (3 motores en paralelo) |
| **ESC Motor Derecho** | **M2** | **GPIO 13** | Naranja (`#D35400`) | Control PWM Tracción Derecha (3 motores en paralelo) |
| **Servo Delantero Izquierdo** | **S1** | **GPIO 18** | Verde (`#27AE60`) | Dirección rueda delantera izquierda |
| **Servo Delantero Derecho** | **S2** | **GPIO 19** | Verde Azulado (`#16A085`) | Dirección rueda delantera derecha |
| **Servo Trasero Izquierdo** | **S3** | **GPIO 21** | Púrpura (`#8E44AD`) | Dirección rueda trasera izquierda (Ackermann contrafase) |
| **Servo Trasero Derecho** | **S4** | **GPIO 22** | Magenta (`#9B59B6`) | Dirección rueda trasera derecha (Ackermann contrafase) |

---

## 2. Esquema 2: Subsistema de Tracción 6x6 (2 ESCs + 6 Motores DC)

Diagrama de cableado detallado para los 6 motores DC de tracción. Muestra cómo los 3 motores del costado izquierdo y los 3 motores del costado derecho se ramifican en paralelo hacia las salidas de potencia `M+` y `M-` de sus respectivos ESCs bidireccionales.

![Diagrama Tracción Motores](/home/jlameda/.gemini/antigravity/brain/e3db32e4-bfca-4eef-84a5-4d985584dddd/schematic_motors_traction.png)

- **Archivos fuente:** [`docs/schematic_motors_traction.svg`](file:///home/jlameda/Documents/Mars-Rover-Proyect/docs/schematic_motors_traction.svg) | [PNG](file:///home/jlameda/Documents/Mars-Rover-Proyect/docs/schematic_motors_traction.png) | [PDF](file:///home/jlameda/Documents/Mars-Rover-Proyect/docs/schematic_motors_traction.pdf)

### Especificación de Cableado de Motores:
1. **Conexión en Paralelo:** 
   - **Banco Izquierdo:** Motores FL (Delantero Izq), ML (Medio Izq) y BL (Trasero Izq) van conectados juntos al ESC 1 (`M+` y `M-`).
   - **Banco Derecho:** Motores FR (Delantero Der), MR (Medio Der) y BR (Trasero Der) van conectados juntos al ESC 2 (`M+` y `M-`).
2. **Regla de Inversión de Giro:** Si alguna rueda gira en sentido opuesto al resto del vehículo al avanzar, basta con intercambiar sus dos cables directamente en los bornes de ese motor específico.
3. **Control y Señal:** Los ESCs reciben señal PWM estándar de 50 Hz (1000 µs = reversa máxima, 1500 µs = neutro/freno, 2000 µs = avance máximo).

---

## 3. Esquema 3: Distribución de Energía y Doble Riel (Batería, UBEC y GND)

Arquitectura de alimentación eléctrica desacoplada con doble riel para proteger la lógica de control frente a caídas de tensión y ruido inductivo de los motores.

![Diagrama Fuente de Alimentación](/home/jlameda/.gemini/antigravity/brain/e3db32e4-bfca-4eef-84a5-4d985584dddd/schematic_power_supply.png)

- **Archivos fuente:** [`docs/schematic_power_supply.svg`](file:///home/jlameda/Documents/Mars-Rover-Proyect/docs/schematic_power_supply.svg) | [PNG](file:///home/jlameda/Documents/Mars-Rover-Proyect/docs/schematic_power_supply.png) | [PDF](file:///home/jlameda/Documents/Mars-Rover-Proyect/docs/schematic_power_supply.pdf)

### Reglas Críticas de Energía:
1. **Topología de Doble Riel:**
   - **Riel de Potencia (+7.4V Batería):** Alimenta directamente los 2 ESCs de tracción a través del interruptor y fusible de 20A.
   - **Riel Regulado (+5.0V / 5A UBEC):** Alimenta exclusivamente a los 4 servomotores MG996R y al pin `5V / VIN` del ESP32.
2. **Bus de Masa Común en Estrella (GND):**
   - Todos los retornos de masa (Batería `-`, UBEC `GND`, ESP32 `GND`, ESCs `GND`, Servos `GND`) se conectan a un único punto central de tierra para eliminar bucles de masa (*ground loops*) e interferencias electromagnéticas (EMI).
3. **Protección:** Fusible automotriz de 20A instalado inmediatamente tras el terminal positivo de la batería LiPo.
