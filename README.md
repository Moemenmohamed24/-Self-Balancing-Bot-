🤖 Self-Balancing Robot
🛠️ Hardware Components:
MPU6050 – For tilt angle measurement (gyroscope + accelerometer).

L293D Motor Driver – To control the two DC motors.

2 × DC Motors – Used for balance and movement.

ESP32-WROOM – Main microcontroller for processing and control.

2 × Dot Matrix Displays – For visual feedback or expression (e.g., eyes or status).

Boost Converters – To stabilize voltage levels for different components.

Lithium Battery – Power source.

Electrolytic Capacitor – For voltage smoothing and noise reduction.

💾 Software Implementation:
Implemented a PID controller using the PID library to maintain balance by adjusting motor speed based on tilt angle feedback from the MPU6050.
