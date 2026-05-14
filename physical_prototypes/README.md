# Prototipi Fisici e Test Reali

Cartella per progettazione, costruzione e test sul campo dei prototipi fisici del drone.

---

## LED Stabilization Test (`led_stabilization_test/`)

Sketch Arduino per validare la logica PID **senza motori**: 4 LED PWM sostituiscono i motori, la luminosità rappresenta la potenza.

### Hardware richiesto
| Componente | Collegamento |
|---|---|
| Waveshare ESP32-S3-Zero | — |
| MPU-6050 | SDA→GPIO 8, SCL→GPIO 9, VCC→3.3V |
| LED Front-Left | GPIO 2 (PWM) + 220Ω → GND |
| LED Front-Right | GPIO 3 (PWM) + 220Ω → GND |
| LED Rear-Left | GPIO 4 (PWM) + 220Ω → GND |
| LED Rear-Right | GPIO 5 (PWM) + 220Ω → GND |

### Setup Arduino IDE
- Board: `ESP32S3 Dev Module`
- Upload speed: `921600`
- USB Mode: `USB-OTG (TinyUSB)`
- Flash Size: `4MB`

### Comportamento atteso
- Board in piano → tutti 4 LED alla stessa luminosità (base throttle)
- Inclinazione pitch → LED anteriori/posteriori si bilanciano
- Inclinazione roll → LED destra/sinistra si bilanciano
- PID riporta al livello → fade progressivo di ritorno

### Output Serial (115200 baud)
CSV: `t_ms, pitch, roll, pid_pitch, pid_roll, FL, FR, RL, RR` — plottabile con Arduino Serial Plotter.

### Parametri PID (tuning conservativo hovering)
- `KP = 1.8` · `KI = 0.05` · `KD = 0.8`
- Complementary filter: 98% gyro + 2% accel

---

## Contenuti previsti
- Schemi elettrici e layout componenti
- Specifiche hardware (motori, ESC)
- Log dei test di volo reali
- Script di calibrazione hardware