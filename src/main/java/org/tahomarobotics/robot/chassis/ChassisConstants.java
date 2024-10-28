package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;

import java.util.List;

import static edu.wpi.first.units.Units.*;

public class ChassisConstants {
    // Physical Properties

    /**
     * Horizontal width from robot center to wheel center
     */
    private static final Distance HALF_TRACK_WIDTH = Meters.of(0.52705).divide(2);
    /**
     * Vertical length from robot center to wheel center
     */
    private static final Distance HALF_WHEEL_BASE = Meters.of(0.52705).divide(2);
    /**
     * Diagonal radius of the robot center to wheel center
     */
    public static final Distance DRIVE_RADIUS = Meters.of(Math.hypot(HALF_TRACK_WIDTH.in(Meters), HALF_WHEEL_BASE.in(Meters)));
    /**
     * Mass of the robot
     */
    private static final Mass MASS = Kilograms.of(25.33313);

    // Module Properties

    private static final Distance WHEEL_RADIUS = Meters.of(0.05), // TODO: Use a measured or tuned number for this.
            WHEEL_CIRCUMFERENCE = WHEEL_RADIUS.times(Math.PI * 2);

    private static final double DRIVE_REDUCTION = MK4i.L2_REDUCTION;
    private static final double STEER_REDUCTION = MK4i.STEER_REDUCTION;

    static final Per<DistanceUnit, AngleUnit> DRIVE_POSITION_COEFFICIENT =
            WHEEL_CIRCUMFERENCE.times(DRIVE_REDUCTION).divide(Rotation.one());
    static final Per<LinearVelocityUnit, AngularVelocityUnit> DRIVE_VELOCITY_COEFFICIENT =
            WHEEL_CIRCUMFERENCE.times(DRIVE_REDUCTION).per(Second).divide(RotationsPerSecond.one());

    // Current Limits

    private static final Current DRIVE_SUPPLY_CURRENT_LIMIT = Amps.of(55);
    private static final Current DRIVE_STATOR_CURRENT_LIMIT = Amps.of(120);
    private static final Current STEER_SUPPLY_CURRENT_LIMIT = Amps.of(20);
    private static final Current STEER_STATOR_CURRENT_LIMIT = Amps.of(30);

    // Physical Limits

    //    public static final LinearAcceleration TRANSLATIONAL_ACCELERATION_LIMIT = MetersPerSecondPerSecond.of(6);
    //    public static final AngularAcceleration ANGULAR_ACCELERATION_LIMIT = RadiansPerSecondPerSecond.of(
    //            TRANSLATIONAL_ACCELERATION_LIMIT.in(MetersPerSecondPerSecond)
    //                    * (1 /* Radian */ / DRIVE_RADIUS.in(Meters))
    //    );
    public static final LinearAcceleration AVERAGE_MODULE_ACCELERATION_LIMIT = MetersPerSecondPerSecond.of(6);

    // Calculated Motor Constants

    private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);

    private static final double MAX_VELOCITY_MULTIPLIER = 1.1;
    public static final LinearVelocity MAX_TRANSLATIONAL_VELOCITY = MetersPerSecond.of(
            DRIVE_MOTOR.freeSpeedRadPerSec // Radians / Second
                    * (WHEEL_RADIUS.in(Meters) /* Meters / Radian */)
                    * DRIVE_REDUCTION * MAX_VELOCITY_MULTIPLIER
    );
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(
            MAX_TRANSLATIONAL_VELOCITY.in(MetersPerSecond)
            * (1 /* Radian */ / DRIVE_RADIUS.in(Meters))
    );

    private static final Per<VoltageUnit, LinearVelocityUnit> KV = VoltsPerMeterPerSecond.ofNative(
            DRIVE_MOTOR.KvRadPerSecPerVolt
            * (Math.PI * 2) // Meters / Radian
    );

    // Device Configurations

    static final CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    static final TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(DRIVE_SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()  // TODO: Tuning
                    .withKP(0.15))
//                    .withKV(kV_DRIVE)
            .withMotorOutput(new MotorOutputConfigs()
//                    .withControlTimesyncFreqHz() TODO: Test
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(120)
                    .withMotionMagicJerk(360))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    static final TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STEER_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(STEER_SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs() // TODO: Tuning
                    .withKP(8.0)
                    .withKI(0.01)
                    .withKD(0.16))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(25)
                    .withMotionMagicJerk(100))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs() {{
                ContinuousWrap = true;
            }})
            .withFeedback(
                    new FeedbackConfigs() {{
                        if (RobotConfiguration.CANIVORE_PHOENIX_PRO) {
                            FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                            RotorToSensorRatio = 1 / DRIVE_REDUCTION;
                        } else {
                            FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                        }
                    }});

    // Module Descriptions

    static final Translation2d[] MODULE_OFFSETS = new Translation2d[]{
            new Translation2d(HALF_WHEEL_BASE.in(Meters), HALF_TRACK_WIDTH.in(Meters)),
            new Translation2d(HALF_WHEEL_BASE.in(Meters), -HALF_TRACK_WIDTH.in(Meters)),
            new Translation2d(-HALF_WHEEL_BASE.in(Meters), HALF_TRACK_WIDTH.in(Meters)),
            new Translation2d(-HALF_WHEEL_BASE.in(Meters), -HALF_TRACK_WIDTH.in(Meters))
    };

    static final List<SwerveModuleDescriptor> MODULE_DESCRIPTORS = List.of(
            new SwerveModuleDescriptor(
                    "Front Left", MODULE_OFFSETS[0],
                    RobotMap.FRONT_LEFT_DRIVE, RobotMap.FRONT_LEFT_STEER, RobotMap.FRONT_LEFT_ENCODER),
            new SwerveModuleDescriptor(
                    "Front Right", MODULE_OFFSETS[1],
                    RobotMap.FRONT_RIGHT_DRIVE, RobotMap.FRONT_RIGHT_STEER, RobotMap.FRONT_RIGHT_ENCODER),
            new SwerveModuleDescriptor(
                    "Back Left", MODULE_OFFSETS[2],
                    RobotMap.BACK_LEFT_DRIVE, RobotMap.BACK_LEFT_STEER, RobotMap.BACK_LEFT_ENCODER),
            new SwerveModuleDescriptor(
                    "Back Right", MODULE_OFFSETS[3],
                    RobotMap.BACK_RIGHT_DRIVE, RobotMap.BACK_RIGHT_STEER, RobotMap.BACK_RIGHT_ENCODER)
    );

    // MK4i Constants

    /**
     * <a href="https://www.swervedrivespecialties.com/products/mk4i-swerve-module">MK4i</a>
     */
    static class MK4i {
        public static final double STEER_REDUCTION = 7d / 150d;

        public static final double L1_REDUCTION = (14d / 50d) * (25d / 19d) * (15d / 45d);
        public static final double L2_REDUCTION = (14d / 50d) * (27d / 17d) * (15d / 45d);
        public static final double L3_REDUCTION = (14d / 50d) * (28d / 16d) * (15d / 45d);
    }
}
