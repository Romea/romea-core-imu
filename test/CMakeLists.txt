find_package(GTest REQUIRED)
include(romea_add_gtest)

romea_add_gtest(test_imu test_imu.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_triad_algorithm test_triad_algorithm.cpp LIBRARIES ${PROJECT_NAME})

# romea_add_gtest(test_roll_pitch_kalman_estimator test_roll_pitch_kalman_estimator.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_zero_velocity_estimator test_zero_velocity_estimator.cpp LIBRARIES ${PROJECT_NAME})
