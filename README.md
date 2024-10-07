## ROMEA Core IMU Library

## IMU Data Processing

This library provides a comprehensive suite of tools for handling data from Inertial Measurement Units (IMUs), including:

- **6 Degrees of Freedom (6 DOF) IMU**

  Combines a 3-axis accelerometer and a 3-axis gyroscope to measure linear acceleration and angular velocity.

- **9 Degrees of Freedom (9 DOF) IMU**

  Enhances the 6 DOF IMU by adding a 3-axis magnetometer, enabling more comprehensive orientation tracking through magnetic field sensing.

- **Attitude and Heading Reference System (AHRS**)

  Leverages data from the 9 DOF IMU to compute complete 3D orientation, including roll, pitch, and yaw angles.

- **Vertical Reference Unit (VRU)**

  Specializes in calculating roll and pitch to provide stable vertical orientation, making it ideal for systems that require accurate inclination measurements.

## Algorithms for IMU Data Processing

This library offers several essential algorithms for processing and  fusing IMU data to estimate orientation, position, and motion states:

- **Triad Algorithm**
  A simple and efficient method to estimate orientation using two  non-collinear reference vectors, typically from the accelerometer and  magnetometer. By aligning these reference vectors with known Earth-based vectors (gravity and magnetic north), the algorithm provides a basic  yet effective orientation estimate.
- **Zero Velocity Update (ZUPT)**
  Commonly used in applications like Pedestrian Dead Reckoning (PDR) or scenarios  where an IMU is attached to a moving object. ZUPT detects stationary  periods (e.g., a foot at rest during walking) to reduce velocity drift,  correcting integration errors when calculating position from  acceleration data.
- **Roll and Pitch Kalman Filter**
  A Kalman Filter is used to optimally fuse noisy sensor data, producing  smooth and accurate orientation estimates. This implementation focuses  on estimating **roll** (rotation about the x-axis) and **pitch** (rotation about the y-axis), using a combination of accelerometer and gyroscope data for reliable attitude estimation.
