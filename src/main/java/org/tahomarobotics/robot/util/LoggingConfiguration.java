package org.tahomarobotics.robot.util;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.wpilibj.DataLogManager;
import org.tahomarobotics.robot.Robot;

public class LoggingConfiguration {
    /**
     * Configures Epilogue and the DataLogManager
     * to log annotations and NetworkTables to file.
     * @param robot The robot instance
     */
    public static void configureEpilogue(Robot robot) {
        // Attempts to log to a USB first at /u/logs, otherwise the RIO itself in /home/lvuser/logs.
        // Simulation logs are stored in logs/
        // Console is not logged in simulation
        DataLogManager.start();

        Epilogue.bind(robot);
        Epilogue.configure(configuration -> {
            // TODO: Custom Logger for filtering non-essential values from NetworkTables when FMS is attached.
        });
    }
}
