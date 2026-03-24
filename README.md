# 🥚 MEGAJAVCHACK Automatic Egg Sorting System

An automated egg sorting system using ESP32, HX711 load cell, and servo motors.  
This system classifies eggs into 6 categories: XS, Small, Medium, Large, XL, Jumbo.

---

## 🚀 Features

- ⚡ Real-time egg detection and classification
- ⚖️ Accurate weight measurement using HX711
- 🤖 Automated sorting via servo gates
- 📟 LCD display for egg type and final weight
- 📡 ESP-NOW communication between controllers
- 📊 Expandable for web dashboard monitoring

---

## 🧠 System Architecture

- ESP32 #1 → Weight detection (HX711 + LCD + flip servo)
- ESP32 #2 → Sorting mechanism (6 servo gates)

---

## 🔌 Pin Configuration

### ESP32 #1 (Weighing Unit)
| Component | Pin |
|----------|-----|
| HX711 DT | GPIO 4 |
| HX711 SCK | GPIO 5 |
| Flip Servo | GPIO 13 |
| LCD (I2C) | SDA: 21 / SCL: 22 |

### ESP32 #2 (Sorting Unit)
| Egg Type | Servo Pin |
|---------|----------|
| XS      | GPIO 13 |
| Small   | GPIO 12 |
| Medium  | GPIO 14 |
| Large   | GPIO 27 |
| XL      | GPIO 26 |
| Jumbo   | GPIO 25 |

---

## ⚖️ Egg Classification

| Type   | Weight Range |
|--------|-------------|
| XS     | 40 – 49.99 g |
| Small  | 50 – 54.99 g |
| Medium | 55 – 59.99 g |
| Large  | 60 – 64.99 g |
| XL     | 65 – 69.99 g |
| Jumbo  | 70+ g |

---

## 🛠️ Build Steps

1. Connect all components based on wiring table
2. Upload code to ESP32 boards
3. Power both ESP32 units
4. Calibrate the load cell
5. Start placing eggs

---

## ⚙️ Calibration Procedure

1. Remove all weight from the load cell
2. Power on system → wait for "TARE COMPLETE"
3. Place known weight (e.g., 100g)
4. Adjust calibration factor in code until accurate
5. Repeat until stable reading achieved

---

## 📸 System Preview

![System](images/system_design.png)

---

## 🧑‍💻 Developers

MEGAJAVCHACK Team  
Marrisa • Emil Jon • Grace • Akissah • JonhRogiel • Anna Mae • Vhan • Carms • Henre • Cristina Marie • Cristina • Karyl

---

## 📌 Future Improvements

- Web dashboard (real-time monitoring)
- AI-based egg quality detection
- Mobile app integration
