package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class RobotMap {
    public static final String CANIVORE_NAME = RobotBase.isSimulation() ? "" : "CANivore";

    public static final int PIGEON = 0;

    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_RIGHT_DRIVE = 2;
    public static final int BACK_LEFT_DRIVE = 3;
    public static final int BACK_RIGHT_DRIVE = 4;

    public static final int FRONT_LEFT_STEER = 11;
    public static final int FRONT_RIGHT_STEER = 12;
    public static final int BACK_LEFT_STEER = 13;
    public static final int BACK_RIGHT_STEER = 14;

    public static final int FRONT_LEFT_ENCODER = 21;
    public static final int FRONT_RIGHT_ENCODER = 22;
    public static final int BACK_LEFT_ENCODER = 23;
    public static final int BACK_RIGHT_ENCODER = 24;
}
