# Prototipi Fisici e Test Reali

Cartella per progettazione, costruzione e test sul campo dei prototipi fisici del drone.

---

## LED Stabilization Test (`led_stabilization_test/`)

Sketch Arduino per validare la logica PID **senza motori**: 4 LED PWM sostituiscono i motori, la luminosità rappresenta la potenza.

### Hardware richiesto
| Componente | Collegamento |
|---|---|
| Arduino Uno/Nano | — |
| MPU-6050 | SDA→A4, SCL→A5, VCC→3.3V |
| LED Front-Left | pin 9 (PWM) + 220Ω |
| LED Front-Right | pin 10 (PWM) + 220Ω |
| LED Rear-Left | pin 5 (PWM) + 220Ω |
| LED Rear-Right | pin 6 (PWM) + 220Ω |

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