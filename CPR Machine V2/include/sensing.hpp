/**
 * @file sensing.hpp
 * @brief Sensing functions for the CPR machine.
 *
 * This header file contains function declarations for sensing-related operations
 * in the CPR machine. These functions are intended to be used directly in the main
 * state machine loop and handle the acquisition and processing of sensor data
 * from force sensors, position sensors, and other diagnostic inputs.
 *
 * @date 2026-05-10
 * @version 1.0
 */

#ifndef SENSING_HPP
#define SENSING_HPP

/**
 * @brief Initializes all sensors.
 *
 * This function sets up sensor hardware, calibrates sensors, and initializes sensor communication protocols. This includes zeroing the force sensor(s), IMU, and ToF.
 *
 * @return int PLACEHOLDER, will eventually return error state
 */
int init_sensors();

/**
 * @brief Checks the battery state.
 *
 * This function reads the battery voltage and status to ensure sufficient power for operation.
 *
 * @return int PLACEHOLDER, will eventually return error state
 */
int battery_check();

/**
 * @brief Reads sensors during mechanical zeroing.
 *
 * This function reads all sensors relevant to the mechanical zeroing process, such as position sensors and force sensors, to determine when the end effector has reached the mechanical zero position. This function is a wrapper for the un-exposed read_sensors() function, and passes in the required thesholds for error detection into read_sensors().
 *
 * @return int PLACEHOLDER, will eventually return error state
 */
int read_sensors_mech_zeroing();

/**
 * @brief Reads sensors during fine zeroing.
 *
 * This function reads all sensors relevant to the fine zeroing process, such as position sensors and force sensors, to determine when the end effector has reached the fine zero position. This function is a wrapper for the un-exposed read_sensors() function, and passes in the required thesholds for error detection into read_sensors().
 *
 * @return int PLACEHOLDER, will eventually return error state
 */
int read_sensors_fine_zeroing();

/**
 * @brief Reads sensors during compressions.
 *
 * This function reads all sensors relevant to the compression process, such as position sensors and force sensors, to ensure that the setpoint is being tracked accurately and that applied force stays within safe limits. This function is a wrapper for the un-exposed read_sensors() function, and passes in the required thesholds for error detection into read_sensors().
 *
 * @return int PLACEHOLDER, will eventually return error state
 */
int read_sensors_compressions();

/**
 * @brief Checks for sensor errors and validates sensor data.
 *
 * This function takes in sensor readings and performs error detection using input thresholds. This is also done internally via read_sensors(), but is exposed in case additional error check calls are required.
 *
 * @return int PLACEHOLDER, will eventually return error state
 */
int check_sensor_error();

/**
 * @brief Sets the chest position reference point.
 *
 * This function establishes the current end effector position as the zero reference for chest position setpoints during compressions.
 *
 * @return int PLACEHOLDER, will eventually return error state
 */
int zero_chest_position();

#endif // SENSING_HPP
