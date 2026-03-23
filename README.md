# frankenbot
An arduino robot that uses ultrasonic sensor to avoid obstacles.

This project describes the construction process of a UGV (Unmanned Ground Vehicle) using Arduino, which moves autonomously by avoiding obstacles. As detailed below, we used an Arduino Uno to implement the logic, DC motors for movement and an Ultrasonic sensor combined with a servo motor to detect obstacles.

# Architecture
The UGV architecture is based on an Arduino Uno R4 WiFi board connected to an Ultrasonic sensor and a servo motor to detect obstacles and their distance. We have connected 4 DC motors to 2 L293D motor drivers, using an external battery to power the motors.

# Equipment
**Arduino Uno R4 WiFi:** Features an ARM Cortex M4 microprocessor. It uses digital pins for logic and a 5V power output for the sensor and servo. It is powered by a 9V battery via the barrel jack, which the board regulates down to 5V.

**Ultrasonic HC-SR04:** Operates like sonar, emitting high-frequency sound that reflects off surfaces. By measuring the return time and knowing the speed of sound, it calculates distance.

**L293D Motor Driver:** An integrated circuit that allows the Arduino to control up to 2 DC motors (per chip), as the Arduino cannot provide enough current directly. It operates between 4.5V and 36V.

**MG996R Servo Motor:** Used to rotate the ultrasonic sensor left and right to scan for paths when an obstacle is detected ahead.

**N20 Micro Metal Gear Motor:** Four 6V DC motors with internal metal gearboxes are used for high torque and durability, reaching speeds of 300 RPM.

# Circuit
<img width="1137" height="793" alt="circuit" src="https://github.com/user-attachments/assets/b66b34d6-7e9c-4385-96f3-79b2c6ff6838" />

# Logic Flow
![frankenbot](https://github.com/user-attachments/assets/eee9752c-5f29-4d7c-a44c-c4b53ffc971b)

# It's alive!
![5](https://github.com/user-attachments/assets/b90f5061-4a00-4aca-8252-b6ca83f1d2e6)
![4](https://github.com/user-attachments/assets/60698c2f-aae0-4942-ae35-2d6450d8e0fa)
![3](https://github.com/user-attachments/assets/430951ab-844b-46ee-81e6-59f7b2149fa9)
![2](https://github.com/user-attachments/assets/d4d537f3-6fd3-426b-a503-a7fc607bb82d)
![1](https://github.com/user-attachments/assets/8d28b3e8-b2bb-4621-b044-2f35f1c95382)
