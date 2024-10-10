## ROMEA Core IMU Library

This library provides a comprehensive suite of tools for handling data from Inertial Measurement Units (IMUs), including:

- **6 Degrees of Freedom (6 DOF) IMU**
  Combines a 3-axis accelerometer and a 3-axis gyroscope to measure linear acceleration and angular velocity.
- **9 Degrees of Freedom (9 DOF) IMU**
  Enhances the 6 DOF IMU by adding a 3-axis magnetometer, enabling more comprehensive orientation tracking through magnetic field sensing.
- **Attitude and Heading Reference System (AHRS)**
  Leverages data from the 9 DOF IMU to compute complete 3D orientation, including roll, pitch, and yaw angles.
- **Vertical Reference Unit (VRU)**
  Specializes in calculating roll and pitch to provide stable vertical orientation, making it ideal for systems that require accurate inclination measurements.

This library also offers several essential algorithms for processing and  fusing IMU data to estimate orientation, position, and motion states:

- **Triad Algorithm**
  A simple and efficient method to estimate orientation using two  non-collinear reference vectors, typically from the accelerometer and  magnetometer. By aligning these reference vectors with known Earth-based vectors (gravity and magnetic north), the algorithm provides a basic  yet effective orientation estimate.
- **Zero Velocity Update (ZUPT)**
  Commonly used in applications like Pedestrian Dead Reckoning (PDR) or scenarios  where an IMU is attached to a moving object. ZUPT detects stationary  periods (e.g., a foot at rest during walking) to reduce velocity drift,  correcting integration errors when calculating position from  acceleration data.
- **Roll and Pitch Kalman Filter**
  A Kalman Filter is used to optimally fuse noisy sensor data, producing  smooth and accurate orientation estimates. This implementation focuses  on estimating **roll** (rotation about the x-axis) and **pitch** (rotation about the y-axis), using a combination of accelerometer and gyroscope data for reliable attitude estimation.

## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://raw.githubusercontent.com/Romea/romea-core-imu/refs/heads/main/romea_imu_public.repos
5. vcs import src < romea_filter_public.repos
6. build packages
   - catkin build for ROS1
   - colcon build for ROS2
7. create your application using this library

## **Contributing**

If you'd like to contribute to Project Title, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

Project Title is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The Romea Core IMU library was developed by **Jean Laneurit** in the context of various research projects carried out at INRAE.

## **Contact**

If you have any questions or comments about Romea Core IMU library, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** 