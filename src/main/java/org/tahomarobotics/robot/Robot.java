// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tahomarobotics.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.util.LoggingConfiguration;

@Logged
public class Robot extends TimedRobot {
    private static final Logger logger = LoggerFactory.getLogger(Robot.class);

    public Robot() {
        LoggingConfiguration.configureEpilogue(this);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}
    @Override
    public void disabledPeriodic() {}
    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {}
    @Override
    public void autonomousPeriodic() {}
    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {}
    @Override
    public void teleopPeriodic() {}
    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
    @Override
    public void testPeriodic() {}
    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
