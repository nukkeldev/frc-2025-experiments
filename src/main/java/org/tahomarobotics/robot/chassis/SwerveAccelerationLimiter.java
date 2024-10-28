package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

public class SwerveAccelerationLimiter {
    private final SwerveModuleState[] prevStates;
    private double prevTime;

    public SwerveAccelerationLimiter(SwerveModuleState[] initialStates) {
        prevStates = initialStates.clone();
        prevTime = MathSharedStore.getTimestamp();
    }

    public void limit(SwerveModuleState[] states) {
        double currentTime = MathSharedStore.getTimestamp();
        double dT = currentTime - prevTime;
        prevTime = currentTime;

        double[] accelerations = new double[states.length];

        double averageAcceleration = 0;
        boolean brakeMode = true;
        for (int i = 0; i < states.length; i++) {
            brakeMode &= states[i].speedMetersPerSecond == 0.0;
            accelerations[i] = (states[i].speedMetersPerSecond - prevStates[i].speedMetersPerSecond) / dT;
            averageAcceleration += Math.abs(accelerations[i]);
        }
        averageAcceleration /= states.length;

        double scale = averageAcceleration > ChassisConstants.AVERAGE_MODULE_ACCELERATION_LIMIT.in(MetersPerSecondPerSecond) ?
                ChassisConstants.AVERAGE_MODULE_ACCELERATION_LIMIT.in(MetersPerSecondPerSecond) / averageAcceleration : 1.0;

        for (int i = 0; i < states.length; i++) {
            states[i].speedMetersPerSecond = brakeMode ? 0.0 : prevStates[i].speedMetersPerSecond + scale * accelerations[i] * dT;
            prevStates[i].speedMetersPerSecond = states[i].speedMetersPerSecond;
        }
    }
}
