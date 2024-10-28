package org.tahomarobotics.robot;

import edu.wpi.first.units.measure.Frequency;

import static edu.wpi.first.units.Units.Hertz;

public class RobotConfiguration {
    public static final boolean CANIVORE_PHOENIX_PRO = true;
    public static final Frequency CANIVORE_STATUS_SIGNAL_UPDATE_FREQUENCY = Hertz.of(250);
    public static final Frequency CANIVORE_CONTROL_REQUEST_UPDATE_FREQUENCY = Hertz.of(1000);
    public static final Frequency RIO_STATUS_SIGNAL_UPDATE_FREQUENCY = Hertz.of(100);
    public static final Frequency RIO_CONTROL_REQUEST_UPDATE_FREQUENCY = Hertz.of(100);
}
