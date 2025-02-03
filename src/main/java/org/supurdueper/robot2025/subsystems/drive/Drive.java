package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import org.supurdueper.robot2025.subsystems.drive.generated.DriveTelemetry;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily be used in
 * command-based projects.
 */
public class Drive extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private Notifier m_simNotifier = null;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    private final PIDController m_pathXController = new PIDController(10, 0, 0);
    private final PIDController m_pathYController = new PIDController(10, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(7, 0, 0);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private DriveTelemetry telemetry = new DriveTelemetry(MaxSpeed);

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If they
     * need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public Drive(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        registerTelemetry(telemetry::telemeterize);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If they
     * need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is 250
     *     Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public Drive(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        registerTelemetry(telemetry::telemeterize);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If they
     * need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is 250
     *     Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form [x, y, theta]ᵀ, with
     *     units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y, theta]ᵀ, with
     *     units in meters and radians
     * @param modules Constants for each specific module
     */
    public Drive(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        registerTelemetry(telemetry::telemeterize);
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(() -> getState().Pose, this::resetPose, this::followPath, true, this, trajLogger);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);
        targetSpeeds.omegaRadiansPerSecond +=
                m_pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

        setControl(m_pathApplyFieldSpeeds
                .withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY()));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        DogLog.log("BatteryVoltage", RobotController.getBatteryVoltage());
        DogLog.log("Drive/OdometryPose", getState().Pose);
        DogLog.log("Drive/TargetStates", getState().ModuleTargets);
        DogLog.log("Drive/MeasuredStates", getState().ModuleStates);
        DogLog.log("Drive/MeasuredSpeeds", getState().Speeds);
        if (mapleSimSwerveDrivetrain != null)
            DogLog.log("Drive/SimulationPose", mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
    }

    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                Pounds.of(115),
                Inches.of(30),
                Inches.of(30),
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX60(1),
                1.2,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        Timer.delay(0.1); // wait for simulation to update
        super.resetPose(pose);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still
     * accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still
     * accounting for measurement noise.
     *
     * <p>Note that the vision measurement standard deviations passed into this method will continue to apply to future
     * measurements until a subsequent call to {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form [x, y, theta]ᵀ,
     *     with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(
                visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}
