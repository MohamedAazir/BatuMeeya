# BatuMeeya
This repository contains the design and implementation files of the micromouse BatuMeeya by team LakeBotics. 
<div align="center">
  <img src=https://github.com/user-attachments/assets/cb5f0bc0-32fc-4af6-abba-428dc2ac65bc width="40%"/>
</div>

## ⚙️ Hardware Components

| Component        | Description                              |
|------------------|------------------------------------------|
| STM32 Blue Pill  | Main microcontroller                     |
| MPU6050          | 6-axis gyroscope & accelerometer         |
| VL53L0X x2/3     | Time-of-Flight distance sensors           |
| MX1508 Driver    | Motor driver module                      |
| N20 Motors       | High RPM DC motors with encoders         |
| Ball Caster      | For 3-wheeled stable design              |

## 🔍 Component Descriptions

**🔷 STM32 Blue Pill**  
The STM32 Blue Pill is a low-cost development board based on the ARM Cortex-M3 STM32F103C8T6 microcontroller. It offers multiple GPIOs, timers, ADCs, UART, SPI, and I2C interfaces, making it ideal for embedded applications like Micromouse. It provides enough processing power and peripherals to handle sensor readings, motor control, and decision-making algorithms efficiently.

**📐 MPU6050**  
The MPU6050 is a 6-axis MEMS sensor that combines a 3-axis gyroscope and a 3-axis accelerometer. It is used to track orientation and detect motion in the micromouse, helping maintain stability during movement and ensuring precise turns. Communication is handled via the I2C protocol, and data can be fused for better accuracy using sensor fusion algorithms.

**📏 VL53L0X**  
VL53L0X is a laser-based Time-of-Flight (ToF) distance sensor capable of measuring absolute distances up to 2 meters with high precision. These sensors are used in the front and sides of the micromouse to detect walls and obstacles. Their fast response time and narrow detection beam make them suitable for maze exploration and wall-following behavior.

**⚙️ MX1508 Motor Driver**  
The MX1508 is a dual-channel DC motor driver module capable of driving two motors simultaneously. It supports PWM speed control and direction control, making it a perfect choice for controlling the micromouse’s differential drive motors. It's compact, efficient, and works well with low-voltage motors like the N20.

**🌀 N20 Motors with Encoders**  
N20 motors are compact brushed DC motors with a gearbox and optional magnetic rotary encoders. These motors provide enough torque and speed for quick navigation through mazes. Encoders are used to track wheel rotations, enabling accurate speed and position feedback necessary for precise control.

**🛞 Ball Caster**  
A ball caster is a passive wheel that supports the third point of contact in a 3-wheeled robot design. It allows smooth, multidirectional movement and helps maintain balance while the two powered wheels handle the actual driving and steering. Its low friction makes it ideal for lightweight and agile designs like the micromouse.

<div align="center">
  <img src=https://github.com/user-attachments/assets/cb9611af-9275-4d43-bd03-d670acaf8d6b width="40%"/>
</div>

## 💻 Firmware Implementation

All firmware development was done using **STM32CubeIDE**, which provides an integrated environment for configuring peripherals, writing embedded C code, and flashing the firmware to the STM32 Blue Pill. HAL (Hardware Abstraction Layer) libraries were used for interfacing with peripherals such as timers, UART, I2C, and GPIOs, allowing efficient and readable code development.

### 🧠 Maze Solving Algorithm

To navigate the maze, we implemented a **modified Flood Fill algorithm**. This algorithm assigns cost values to maze cells and updates them dynamically as the mouse explores unknown paths. The robot chooses the neighboring cell with the lowest cost, allowing it to find the shortest path to the center. Our modified version improves exploration efficiency by minimizing unnecessary backtracking and favoring unexplored paths intelligently.

This approach ensures fast, memory-efficient decision-making suitable for real-time execution on the STM32 microcontroller.
