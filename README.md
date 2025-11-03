Arduino UNO line follower with 2 IR sensors and HC-SR04 obstacle stop. Interrupt-driven echo, PWM speed control, no extra libraries. Pins: IR L=12, IR R=11, TRIG=2, ECHO=3 (INT1), L298N ENA=6 / IN1=7 / IN2=8, ENB=5 / IN3=9 / IN4=10.

Line Follower Robot with Ultrasonic Obstacle Stop (ATmega328P / Arduino UNO)

A simple two/Four-wheel line follower that uses two IR sensors for line detection and an HC-SR04 ultrasonic sensor for obstacle stopping. Motor speed is PWM-controlled; distance is measured using an interrupt-driven echo for non-blocking reads.

Works on Arduino UNO / Nano (ATmega328P). No external libraries required.

Features

🚗 Differential drive line following (2 IR sensors: left & right)

🧱 Automatic stop when obstacle ≤ OBSTACLE_THRESHOLD cm

⚡ Non-blocking distance read using external interrupt on ECHO_PIN

🎚️ PWM speed control with easy tuning (MOTOR_SPEED)

🧪 Serial debug output (9600 bps)

Hardware

Arduino UNO / Nano (ATmega328P)

Motor driver: L298N 

2x DC gear motors (left/right)
if u use 4 motors.. couple them

2x IR reflective sensors (digital output type)

HC-SR04 ultrasonic sensor

Power: 7–12 V battery pack (motors) + 5 V for logic

Pin Mapping
Ultrasonic (HC-SR04)
Signal	Pin
TRIG	2
ECHO	3 (INT1)
IR Sensors (digital)
Sensor	Pin	Active Level
Right IR	11	LOW = line detected
Left IR	12	LOW = line detected
Motors (L298N)
Signal	Pin	Notes
ENA (Right enable)	6 (PWM)	enableRightMotor
IN1 (Right dir 1)	7	rightMotorPin1
IN2 (Right dir 2)	8	rightMotorPin2
ENB (Left enable)	5 (PWM)	enableLeftMotor
IN3 (Left dir 1)	9	leftMotorPin1
IN4 (Left dir 2)	10	leftMotorPin2

Ensure ENA/ENB jumpers are removed on L298N so PWM from pins 6/5 actually controls speed.

Code

Your full sketch is in . Key constants:

#define OBSTACLE_THRESHOLD 100   // cm
#define MOTOR_SPEED 180          // 0..255


Movement logic:

Both IR on line (LOW, LOW) → go forward

Right off, left on (HIGH, LOW) → turn left

Right on, left off (LOW, HIGH) → turn right

Both off (HIGH, HIGH) → stop

Obstacle logic:

If 0 < distance ≤ OBSTACLE_THRESHOLD → stop motors and print message

How to Build & Upload
Arduino IDE

Board: Arduino UNO (or Nano / ATmega328P)

Port: Select the correct COM/tty

Open the sketch and click Upload

PlatformIO (optional)
git clone https://github.com/Nafrees9893/Line-follower-with-A.Uno.git
cd Line-follower-with-A.uno

pio run
pio run -t upload
pio device monitor -b 9600

Power & Wiring Tips

Share GND between Arduino, L298N, sensors, and battery negative.

Motors should have their own supply (e.g., 7–12 V) into L298N; Arduino 5 V can power sensors.

Keep ultrasonic away from motor noise sources; route echo/trig wires cleanly.

Tuning Guide

MOTOR_SPEED: Start around 150–180; increase until stable.

OBSTACLE_THRESHOLD: Set based on stopping distance on your surface (e.g., 15–25 cm for tight spaces, 50–100 cm for gentle stops).

IR sensor thresholds: Your sensors output digital HIGH/LOW. If using analog IR later, you’ll need thresholding with analogRead().

Serial Debug

Open Serial Monitor at 9600 bps

You’ll see:

Distance: <value> cm

Obstacle detected, stopping motors

Invalid distance detected (out of 2–400 cm)

No echo received (timeout)

Important Notes (Timer0 / Interrupts)

The sketch changes Timer0 prescaler:

TCCR0B = TCCR0B & B11111000 | B00000010; // prescaler = 8


This raises PWM frequency on pins 5 & 6, but on classic AVR Timer0 also drives millis(), micros(), and delay(). Changing the prescaler can skew timing for those functions. Because this code relies on micros() for distance, if you notice incorrect distance readings or odd delays:

Remove that line (revert to default Timer0), or

Move PWM to Timer2 pins (e.g., use different enable pins) if you want higher PWM frequency without affecting Timer0.

The echo interrupt uses CHANGE to capture both rising and falling edges on ECHO (pin 3/INT1). Ensure no other ISR blocks for long periods.

Troubleshooting

Robot won’t move: Check ENA/ENB jumpers removed, correct power, and that MOTOR_SPEED > 0.

Spins in place: Swap a motor’s direction pins (IN1/IN2 or IN3/IN4) or invert logic.

Always stopping: Ultrasonic too close to chassis? Echoing off the robot. Angle the sensor slightly or extend it forward.

No echo received: Check 5 V power, loose wires, bad ECHO pin connection, or environmental noise.

Distance seems wrong: Remove/adjust Timer0 prescaler line; keep sensor >2 cm and <4 m from obstacles.

IR logic inverted: Some IR modules output LOW on white and HIGH on black. Verify with Serial prints (temporarily log leftIR/rightIR) and flip decisions if needed.

Folder Structure
.
├── README.md
├── code.ino  
└── /hardware
    └── wiring_diagram.png

feuture road map

PID control for smoother line following

3-IR or 5-IR array for better edge detection

Soft-stop / slow-down near obstacles instead of hard stop

Bumper switch as a redundant failsafe

License

MIT License — free to use, modify, and share. Add your name & year.

Credits

Designed & built by Arif M Nafrees.
If this helped, ⭐ star the repo and share your build!
