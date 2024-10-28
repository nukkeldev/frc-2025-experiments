package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;

import java.util.List;

import static edu.wpi.first.units.Units.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class SwerveModule {
    private final Logger logger;

    public final String name;

    @Logged
    private final MutAngle steerOffset;

    // Hardware

    private final TalonFX driveMotor, steerMotor;
    private final CANcoder encoder;

    // State

    private SwerveModuleState desiredState = new SwerveModuleState();

    // Status Signals

    private final StatusSignal<Angle> drivePosition, steerPosition;
    private final StatusSignal<AngularVelocity> driveVelocity, steerVelocity;
    private final StatusSignal<AngularAcceleration> driveAcceleration;

    private final StatusSignal<Current> driveCurrent, steerCurrent;

    // Control Requests

    private final VelocityVoltage driveMotorVelocity = new VelocityVoltage(0)
            .withEnableFOC(RobotConfiguration.CANIVORE_PHOENIX_PRO)
            .withUpdateFreqHz(RobotConfiguration.CANIVORE_CONTROL_REQUEST_UPDATE_FREQUENCY);
    private final PositionDutyCycle steerMotorPosition = new PositionDutyCycle(0)
            .withEnableFOC(RobotConfiguration.CANIVORE_PHOENIX_PRO)
            .withUpdateFreqHz(RobotConfiguration.CANIVORE_CONTROL_REQUEST_UPDATE_FREQUENCY);

    // Constructor

    SwerveModule(SwerveModuleDescriptor descriptor, Angle initialOffset) {
        logger = LoggerFactory.getLogger(SwerveModule.class.getName() + "." + descriptor.name());

        name = descriptor.name();
        steerOffset = initialOffset.mutableCopy();

        driveMotor = new TalonFX(descriptor.driveMotorId(), RobotMap.CANIVORE_NAME);
        steerMotor = new TalonFX(descriptor.steerMotorId(), RobotMap.CANIVORE_NAME);
        encoder = new CANcoder(descriptor.encoderId(), RobotMap.CANIVORE_NAME);

        RobustConfigurator.tryConfigureTalonFX(name + " Drive", driveMotor.getConfigurator(), ChassisConstants.driveMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX(name + " Steer", steerMotor.getConfigurator(), ChassisConstants.steerMotorConfiguration);
        RobustConfigurator.tryConfigureCANcoder(name + " Encoder", encoder.getConfigurator(), ChassisConstants.encoderConfiguration);
        syncSteerOffset();

        // Status Signals

        driveCurrent = driveMotor.getSupplyCurrent();
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAcceleration = driveMotor.getAcceleration();

        steerCurrent = steerMotor.getSupplyCurrent();
        steerPosition = steerMotor.getPosition();
        steerVelocity = steerMotor.getVelocity();

        RobustConfigurator.trySetUpdateFrequencyForAll(name, RobotConfiguration.CANIVORE_STATUS_SIGNAL_UPDATE_FREQUENCY,
                driveCurrent, drivePosition, driveVelocity, driveAcceleration,
                steerCurrent, steerPosition, steerVelocity);
        RobustConfigurator.tryOptimizeBusUsageForAll(name, driveMotor, steerMotor, encoder);
    }

    // Calibration

    void initializeCalibration() {
        // Clear the current offset to get the actual angle
        RobustConfigurator.tryModifyCANcoder(name + " Encoder", encoder.getConfigurator(), config -> config.MagnetSensor.MagnetOffset = 0);
        // Allow for the wheel to be rotated manually
        RobustConfigurator.tryModifyTalonFX(name + " Steer", steerMotor.getConfigurator(), config -> config.MotorOutput.NeutralMode = NeutralModeValue.Coast);
    }

    Angle finalizeCalibration() {
        steerOffset.mut_replace(steerPosition.getValue().unaryMinus());
        syncSteerOffset();
        return steerOffset.copy();
    }

    void cancelCalibration() {
        syncSteerOffset();
    }

    // Configuration

    private void syncSteerOffset() {
        RobustConfigurator.tryModifyCANcoder(name + " Encoder", encoder.getConfigurator(),
                config -> config.MagnetSensor.MagnetOffset = steerOffset.in(Rotations));
    }

    // Getters

    @Logged(name = "position")
    public Distance getDrivePosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity)
                .timesConversionFactor(ChassisConstants.DRIVE_POSITION_COEFFICIENT);
    }

    @Logged(name = "velocity")
    public LinearVelocity getDriveVelocity() {
        return BaseStatusSignal.getLatencyCompensatedValue(driveVelocity, driveAcceleration)
                .timesConversionFactor(ChassisConstants.DRIVE_VELOCITY_COEFFICIENT);
    }

    @Logged(name = "angle")
    public Angle getSteerPosition() {
        return (Angle) BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity);
    }

    void getModuleState(SwerveModuleState out) {
        out.speedMetersPerSecond = getDriveVelocity().in(MetersPerSecond);
        out.angle = new Rotation2d(getSteerPosition());
    }

    void getModulePosition(SwerveModulePosition out) {
        out.distanceMeters = getDrivePosition().in(Meters);
        out.angle = new Rotation2d(getSteerPosition());
    }

    // Periodic

    void periodic() {

    }

    // State

    void setDesiredState(SwerveModuleState state) {
        Angle steer = getSteerPosition();
        state.optimize(new Rotation2d(steer));
        desiredState = state;

        driveMotor.setControl(driveMotorVelocity.withVelocity(
                desiredState.speedMetersPerSecond / ChassisConstants.DRIVE_VELOCITY_COEFFICIENT.in(MetersPerSecond.per(RotationsPerSecond))));
        steerMotor.setControl(steerMotorPosition.withPosition(desiredState.angle.getRotations()));
    }

    void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    // Status Signals

    List<BaseStatusSignal> getCopiesOfStatusSignals() {
        return List.of(
                driveCurrent.clone(), drivePosition.clone(), driveVelocity.clone(), driveAcceleration.clone(),
                steerCurrent.clone(), steerPosition.clone(), steerVelocity.clone()
        );
    }
}
