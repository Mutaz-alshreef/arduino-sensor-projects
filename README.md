# 🔌 Arduino Sensor Projects

This repository contains two Arduino-based sensor projects that use LDR (light sensor), ultrasonic sensor (HC-SR04), and LEDs to create interactive circuits.

---

## 📁 Project 1: LDR + Ultrasonic Sensor Controlled LED

### 🔧 Components Used
- Arduino Uno
- HC-SR04 (3-pin version: VCC, GND, SIG)
- LDR (photoresistor)
- 10kΩ resistor
- LED
- Breadboard & Jumper Wires

### 📷 Circuit Diagram
<img width="896" height="670" alt="image" src="https://github.com/user-attachments/assets/9792bcb3-6d66-424e-8d01-fbafbc898327" />


### ⚙️ How it works
- The LED turns on **only when it's dark (low light)** AND **an object is detected nearby (<20 cm)**.
- LDR is connected to `A1`, LED to `D11`, and HC-SR04 SIG to `D8`.

### 🧠 Code Logic
```cpp
const int ledPin = 4;
const int ldrPin = 2;
const int buttonPin = 0;

bool circuitEnabled = false;

bool lastButtonReading = HIGH;
bool buttonState = HIGH;  // Current debounced state
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ldrPin, INPUT_PULLUP);
  digitalWrite(ledPin, LOW);
}

void loop() {
  bool currentReading = digitalRead(buttonPin);

  // If the button reading has changed, reset debounce timer
  if (currentReading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  // If the reading is stable for > debounceDelay, treat as valid
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If state changed from last known stable state
    if (currentReading != buttonState) {
      buttonState = currentReading;

      // Only toggle on the falling edge (button press)
      if (buttonState == LOW) {
        circuitEnabled = !circuitEnabled;

        if (circuitEnabled) {
          blinkLED(1); // 2 blinks = ON
        } else {
          blinkLED(2); // 1 blink = OFF
        }
      }
    }
  }

  lastButtonReading = currentReading;

  // LDR-controlled LED only when circuit is enabled
  if (circuitEnabled) {
    if (digitalRead(ldrPin) == LOW) {
      digitalWrite(ledPin, HIGH);  // DARK: LED ON
    } else {
      digitalWrite(ledPin, LOW);   // LIGHT: LED OFF
    }
  } else {
    digitalWrite(ledPin, LOW);     // Circuit disabled
  }
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(150);
    digitalWrite(ledPin, LOW);
    delay(150);
  }
}
```
## 📁 Project 2: LDR with LED Control

### 🔧 Components Used
- Arduino Uno
- LDR (photoresistor)
- 10kΩ Resistor
- LED
- Breadboard & Jumper Wires

### 🔌 Circuit Connections
| Component | Arduino Pin |
|----------|-------------|
| LDR      | A1          |
| LED (+)  | D11          |
| LED (–)  | GND (via 220Ω resistor) |
| LDR to GND via resistor |

> 💡 The LDR is connected in a voltage divider setup with a 10kΩ resistor.

### 📷 Circuit Diagram
<img width="1171" height="718" alt="image" src="https://github.com/user-attachments/assets/c7de6ef0-bb03-417c-bed4-92dff3f987df" />


### ⚙️ How It Works
- The LED turns **on** when light level is **low** (darkness).
- When light is bright, the LED turns **off**.

### 🧠 Code

```cpp
const int ledPin = 11;
const int ldrPin = A1;
const int ultrasonicPin = 8; // رجل واحدة للـ HC-SR04 ثلاثي الأرجل

const int ldrThreshold = 500;     // الإضاءة أقل من هذا → ظلام
const int distanceThreshold = 20; // بالسنتيمتر

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(ultrasonicPin, OUTPUT); // للإرسال أولاً
  Serial.begin(9600);
}

void loop() {
  // قراءة حساس الضوء
  int ldrValue = analogRead(ldrPin);

  // قياس المسافة
  long duration;
  float distance;

  pinMode(ultrasonicPin, OUTPUT);
  digitalWrite(ultrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicPin, LOW);

  pinMode(ultrasonicPin, INPUT);
  duration = pulseIn(ultrasonicPin, HIGH);
  distance = duration * 0.034 / 2;
}

  // طباعة القيم لل
