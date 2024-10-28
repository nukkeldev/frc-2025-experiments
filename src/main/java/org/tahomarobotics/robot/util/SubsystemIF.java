package org.tahomarobotics.robot.util;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * An extension of SubsystemBase that adds more lifecycle and logging methods as well as a logger.
 */
public abstract class SubsystemIF extends SubsystemBase {
    /**
     * Subsystem-specific console logger
     */
    protected final Logger logger;

    protected SubsystemIF() {
        logger = LoggerFactory.getLogger(getClass().getSimpleName());
    }

    /**
     * This method should handle anything required to run on code startup, such as zeroing
     * that doesn't require motor control or adding manually-run commands to NetworkTables.
     * @return The instance of this subsystem
     */
    public SubsystemIF initialize() { return this; }

    /**
     * Mirrors {@link IterativeRobotBase#disabledInit()}
     */
    public void onDisabledInit() {}
    /**
     * Mirrors {@link IterativeRobotBase#autonomousInit()}
     */
    public void onAutonomousInit() {}
    /**
     * Mirrors {@link IterativeRobotBase#teleopInit()}
     */
    public void onTeleopInit() {}

    /**
     * The amount of energy used by this subsystem since startup.
     * @return Energy used
     */
    public abstract Energy getEnergyUsed();
    /**
     * The amount of current being used by this subsystem during this control loop.
     * @return Current
     */
    public abstract Current getCurrent();
}
