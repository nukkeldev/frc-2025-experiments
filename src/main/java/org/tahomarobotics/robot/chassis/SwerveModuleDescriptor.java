package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A utility class to consolidate SwerveModule constructor arguments.
 * @param name Name of module; for logging purposes
 * @param offset Translational offset from the center of the chassis to the center of the wheel
 * @param driveMotorId CAN ID of the drive motor
 * @param steerMotorId CAN ID of the steer motor
 * @param encoderId CAN ID of the encoder
 */
record SwerveModuleDescriptor(
        String name,
        Translation2d offset,
        int driveMotorId,
        int steerMotorId,
        int encoderId) {
}