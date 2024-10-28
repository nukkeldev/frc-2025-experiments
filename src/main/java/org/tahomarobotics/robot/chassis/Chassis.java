package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.WrappedFileSyncedData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static edu.wpi.first.units.Units.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Chassis extends SubsystemIF {
    private final static Chassis INSTANCE = new Chassis();

    // Input

    @Logged
    private ChassisSpeeds input = new ChassisSpeeds();

    // Pigeon

    private final Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON);
    private final StatusSignal<Angle> yawSignal = gyro.getYaw();
    @Logged
    private final MutAngle yaw = Radians.zero().mutableCopy();

    // Modules

    @Logged
    private final SwerveModule frontLeftModule;
    @Logged
    private final SwerveModule frontRightModule;
    @Logged
    private final SwerveModule backLeftModule;
    @Logged
    private final SwerveModule backRightModule;

    private final List<SwerveModule> modules;

    private final SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    private final SwerveModuleState[] moduleStates = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    // Swerve

    private final SwerveAccelerationLimiter limiter;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(ChassisConstants.MODULE_OFFSETS);

    private final Thread odometryThread;

    // Calibration

    /**
     * Angular offsets of the modules
     */
    private final WrappedFileSyncedData<Angle[], double[]> calibration;

    // Field

    @Logged
    private final Field2d field = new Field2d();
    @Logged
    private boolean isFieldOriented = false;

    // Initialization

    private Chassis() {
        calibration = new WrappedFileSyncedData<>(
                "SwerveCalibration", new double[4],
                angles -> Arrays.stream(angles).mapToDouble(a -> a.in(Degrees)).toArray(),
                degrees -> Arrays.stream(degrees).mapToObj(Degrees::of).toArray(Angle[]::new)
        );

        frontLeftModule = new SwerveModule(ChassisConstants.MODULE_DESCRIPTORS.get(0), calibration.get()[0]);
        frontRightModule = new SwerveModule(ChassisConstants.MODULE_DESCRIPTORS.get(1), calibration.get()[1]);
        backLeftModule = new SwerveModule(ChassisConstants.MODULE_DESCRIPTORS.get(2), calibration.get()[2]);
        backRightModule = new SwerveModule(ChassisConstants.MODULE_DESCRIPTORS.get(3), calibration.get()[3]);

        modules = List.of(frontLeftModule, frontRightModule, backLeftModule, backRightModule);

        getModuleStates(moduleStates);
        limiter = new SwerveAccelerationLimiter(moduleStates);

        getModulePositions(modulePositions);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics,
                new Rotation2d(), modulePositions,
                new Pose2d(0.0, 0.0, Rotation2d.kZero),
                VecBuilder.fill(0.02, 0.02, 0.02), VecBuilder.fill(0.1, 0.1, 0.1));

        odometryThread = new Thread(this::odometryThread);
        odometryThread.start();
    }

    public static Chassis getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        return this;
    }

    // Modules' Getters

    private void getModulePositions(SwerveModulePosition[] out) {
        for (int i = 0; i < 4; i++) modules.get(i).getModulePosition(out[i]);
    }

    private void getModuleStates(SwerveModuleState[] out) {
        for (int i = 0; i < 4; i++) modules.get(i).getModuleState(out[i]);
    }

    // Modules' Setters

    private void setModuleDesiredStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < desiredStates.length; i++) modules.get(i).setDesiredState(desiredStates[i]);
    }

    // Chassis' Getters

    @Logged
    public Pose2d getPose() {
        synchronized (poseEstimator) {
            return poseEstimator.getEstimatedPosition();
        }
    }

    @Logged
    public Angle getYaw() {
        return yaw;
    }

    // Periodic

    @Override
    public void periodic() {
        modules.forEach(SwerveModule::periodic);

        Pose2d pose = getPose();
        field.setRobotPose(pose);

        if (RobotState.isEnabled()) {
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(input);

            SwerveDriveKinematics.desaturateWheelSpeeds(states, ChassisConstants.MAX_TRANSLATIONAL_VELOCITY);
            limiter.limit(states);

            setModuleDesiredStates(states);
        }
    }

    // Odometry

    private SwerveModulePosition[] calculateModuleDeltas(SwerveModulePosition[] start, SwerveModulePosition[] end) {
        SwerveModulePosition[] moduleDeltas = end.clone();
        for (int i = 0; i < end.length; i++) {
            moduleDeltas[i].distanceMeters -= start[i].distanceMeters;
        }
        return moduleDeltas;
    }

    private void odometryThread() {
        Threads.setCurrentThreadPriority(true, 1);

        final double TIMEOUT = 4 / RobotConfiguration.CANIVORE_STATUS_SIGNAL_UPDATE_FREQUENCY.in(Hertz);

        StatusSignal<Angle> yawSignal = this.yawSignal.clone();

        List<BaseStatusSignal> signals_ = new ArrayList<>();
        signals_.add(yawSignal);
        for (SwerveModule module : modules) {
            signals_.addAll(module.getCopiesOfStatusSignals());
        }

        BaseStatusSignal[] signals = signals_.toArray(BaseStatusSignal[]::new);
        SwerveModulePosition[] positions = Arrays.copyOf(modulePositions, modulePositions.length);

        while (true) {
            BaseStatusSignal.waitForAll(TIMEOUT, signals);
            getModulePositions(positions);

            if (yawSignal.getStatus().isOK()) {
                yaw.mut_replace(yawSignal.getValue());
            } else {
                // Use the angle delta from the kinematics and module deltas
                SwerveModulePosition[] deltas = calculateModuleDeltas(modulePositions, positions);
                Twist2d twist = kinematics.toTwist2d(deltas);

                yaw.mut_plus(twist.dtheta, Radians);
            }

            System.arraycopy(positions, 0, modulePositions, 0, positions.length);

            synchronized (poseEstimator) {
                poseEstimator.update(new Rotation2d(yaw), positions);
            }
        }
    }

    // Energy Diagnostics

    @Override
    public Energy getEnergyUsed() {
        return null;
    }

    @Override
    public Current getCurrent() {
        return null;
    }
}

