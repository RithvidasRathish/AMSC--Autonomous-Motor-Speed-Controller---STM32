# 🚗 AMSC – Autonomous Motor Speed Controller

Embedded firmware for intelligent motor speed control using STM32, FreeRTOS, and real-time sensor data.

---

## 👨‍💻 Author

**Rithvidas Rathish**  
🔗 [LinkedIn Profile](https://www.linkedin.com/in/rithvidas-rathish-embedded-firmware-developer/)

---

## 📌 Project Overview

**AMSC** is an embedded real-time system designed to autonomously control a motor's speed based on throttle input, temperature constraints, and obstacle detection using:

- **STM32 MCU (STM32F4 series)**
- **FreeRTOS** (Preemptive Scheduling)
- **VL53L0X ToF Sensor** – Obstacle Detection
- **DHT20 Sensor** – Temperature & Humidity Sensing
- **ADC with Potentiometer** – Throttle Input
- **PWM (TIM1)** – Motor Speed Control
- **GPIO** – Brake Engagement

---

## 🧠 Features

- ✅ Preemptive multitasking using **FreeRTOS**
- 📏 Obstacle sensing with **VL53L0X** (I2C1)
- 🌡️ Temperature safety check using **DHT20** (I2C2)
- 🎚️ Analog throttle control using ADC
- ⚙️ Smooth PWM-based speed adjustment (TIM1 CH1)
- 🚨 Emergency brake on close-range obstacle
- 🧯 Over-temperature safeguard (PWM limited to 50% above 45°C)

---

## 📦 Architecture

| Task         | Purpose                                          | Peripherals Used   |
|--------------|--------------------------------------------------|--------------------|
| `defaultTask`| Controls PWM & safety logic                      | TIM1, GPIO         |
| `Task02`     | Measures distance using VL53L0X                  | I2C1               |
| `Task03`     | Reads temperature & humidity from DHT20          | I2C2               |
| `Task04`     | Reads ADC throttle value                         | ADC1               |

---

## 🔧 Hardware Setup

| Component         | Connection         |
|------------------|--------------------|
| VL53L0X ToF Sensor| I2C1 (e.g. PB6, PB7) |
| DHT20 Sensor      | I2C2 (e.g. PB10, PB11) |
| Potentiometer     | ADC1 Channel 0 (e.g. PA0) |
| Motor Driver PWM  | TIM1_CH1 (e.g. PA8) |
| Brake GPIO        | Configurable GPIO pin |
| UART Debug        | USART2 (e.g. PA2, PA3) |

---

## 📈 Logic Flow

### Motor Control
- Reads `throttle_percent` from ADC
- Limits duty cycle to 50% if `temperature > 45°C`
- Stops motor and engages brake if `distance < 100mm`

---

## 🚀 Getting Started

### Prerequisites
- STM32CubeIDE / STM32CubeMX
- STM32F4 Development Board (e.g., STM32F446RE)
- USB UART Debug Monitor (e.g., PuTTY)
- Sensors: VL53L0X, DHT20, Potentiometer

### Build & Flash
1. Clone the repository
2. Open project in **STM32CubeIDE**
3. Build the project
4. Flash to the STM32 board
5. Open Serial Terminal @ `115200 baud` for logs

---

## 🖥️ Output Example

```bash
Temperature: 36.52 C, Humidity: 42%
Throttle: 67%
Distance: 133 mm

Distance: 92 mm
Obstacle detected! Motor stopped.
