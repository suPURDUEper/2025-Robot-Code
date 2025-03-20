package org.supurdueper.lib.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import org.supurdueper.lib.utils.GeomUtil;

public class DriveToPose implements SwerveRequest {

    /** The allowable deadband of the request, in m/s. */
    public double Deadband = 0;
    /** The rotational deadband of the request, in radians per second. */
    public double RotationalDeadband = 0;

    /** The rotational deadband of the request, in radians per second. */
    public double distanceTolerance = 0;

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.Velocity;
    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    /**
     * Whether to desaturate wheel speeds before applying. For more information, see the documentation of
     * {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;

    private FieldCentric fieldCentric;
    private PhoenixProfiledPIDController driveController;
    private PhoenixProfiledPIDController thetaController;
    private Pose2d targetPose;
    Translation2d lastSetpointTranslation;
    private double thetaErrorAbs = 0.0;
    private Rotation2d lastSetpointRotation;
    private double lastTime;
    public boolean toReset = false;

    public DriveToPose() {
        this.targetPose = Pose2d.kZero;
        fieldCentric = new FieldCentric();
        driveController = new PhoenixProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        thetaController = new PhoenixProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset(SwerveControlParameters parameters) {
        Pose2d currentPose = parameters.currentPose;
        ChassisSpeeds fieldVelocity = parameters.currentChassisSpeed;
        Translation2d linearFieldVelocity =
                new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
        driveController.reset(
                currentPose.getTranslation().getDistance(targetPose.getTranslation()),
                Math.min(
                        0.0,
                        -linearFieldVelocity
                                .rotateBy(targetPose
                                        .getTranslation()
                                        .minus(currentPose.getTranslation())
                                        .getAngle()
                                        .unaryMinus())
                                .getX()));
        thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
        lastSetpointRotation = targetPose.getRotation();
        lastTime = parameters.timestamp;
        toReset = false;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        if (toReset) {
            reset(parameters);
        }
        if (lastSetpointTranslation == null) {
            lastSetpointTranslation = parameters.currentPose.getTranslation();
        }
        double ffMinRadius = 0.05;
        double ffMaxRadius = 1;
        double currentDistanceError = parameters.currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler = MathUtil.clamp((currentDistanceError - ffMinRadius / (ffMaxRadius - ffMinRadius)), 0.0, 1.0);

        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);
        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(currentDistanceError, 0.0, parameters.timestamp);
        if (currentDistanceError < distanceTolerance) {
            driveVelocityScalar = 0;
        }
        lastSetpointTranslation = new Pose2d(
                        targetPose.getTranslation(),
                        new Rotation2d(Math.atan2(
                                parameters.currentPose.getTranslation().getY()
                                        - targetPose.getTranslation().getY(),
                                parameters.currentPose.getTranslation().getX()
                                        - targetPose.getTranslation().getX())))
                .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        Translation2d driveVelocity = new Pose2d(
                        Translation2d.kZero,
                        new Rotation2d(Math.atan2(
                                parameters.currentPose.getTranslation().getY()
                                        - targetPose.getTranslation().getY(),
                                parameters.currentPose.getTranslation().getX()
                                        - targetPose.getTranslation().getX())))
                .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
                .getTranslation();

        // Handle rotation
        Rotation2d angleToFace = targetPose.getRotation();
        // Calculate theta speed
        double thetaVelocity = thetaController.calculate(
                        parameters.currentPose.getRotation().getRadians(),
                        new TrapezoidProfile.State(
                                targetPose.getRotation().getRadians(),
                                (targetPose.getRotation().minus(lastSetpointRotation)).getRadians()
                                        / (parameters.timestamp - lastTime)),
                        parameters.timestamp)
                + thetaController.getSetpoint().velocity * ffScaler;
        thetaErrorAbs = Math.abs(parameters
                .currentPose
                .getRotation()
                .minus(targetPose.getRotation())
                .getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
        lastSetpointRotation = targetPose.getRotation();
        lastTime = parameters.timestamp;

        DogLog.log("DriveToPose/DistanceMeasured", currentDistanceError);
        DogLog.log("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
        DogLog.log(
                "DriveToPose/ThetaMeasured",
                parameters.currentPose.getRotation().getRadians());
        DogLog.log("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
        DogLog.log("DriveToPose/Setpoint", new Pose2d[] {
            new Pose2d(lastSetpointTranslation, Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
        DogLog.log("DriveToPose/Goal", new Pose2d[] {targetPose});

        return fieldCentric
                .withVelocityX(-driveVelocity.getX())
                .withVelocityY(-driveVelocity.getY())
                .withRotationalRate(thetaVelocity)
                .withDeadband(Deadband)
                .withRotationalDeadband(RotationalDeadband)
                .withDriveRequestType(DriveRequestType)
                .withSteerRequestType(SteerRequestType)
                .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .apply(parameters, modulesToApply);
    }

    public DriveToPose withGoal(Pose2d goal) {
        this.targetPose = goal;
        return this;
    }

    public DriveToPose withTranslationConstraints(LinearVelocity maxSpeed, LinearAcceleration maxAcceleration) {
        driveController.setConstraints(new TrapezoidProfile.Constraints(
                maxSpeed.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond)));
        return this;
    }

    public DriveToPose withHeadingConstraints(AngularVelocity maxSpeed, AngularAcceleration maxAcceleration) {
        thetaController.setConstraints(new TrapezoidProfile.Constraints(
                maxSpeed.in(RadiansPerSecond), maxAcceleration.in(RadiansPerSecondPerSecond)));
        return this;
    }

    /**
     * Modifies the PID gains of the X and Y PID controllers and returns itself.
     *
     * <p>Sets the proportional, integral, and differential coefficients used to maintain the desired location. Users
     * can specify the PID gains to change how aggressively to maintain location.
     *
     * <p>This PID controller operates on translation meters and outputs a target linear rate in meters per second.
     *
     * @param kP The proportional coefficient; must be >= 0
     * @param kI The integral coefficient; must be >= 0
     * @param kD The differential coefficient; must be >= 0
     * @return this object
     */
    public DriveToPose withTranslationPID(double kP, double kI, double kD) {
        driveController.setPID(kP, kI, kD);
        return this;
    }

    /**
     * Modifies the PID gains of the HeadingController parameter and returns itself.
     *
     * <p>Sets the proportional, integral, and differential coefficients used to maintain the desired heading. Users can
     * specify the PID gains to change how aggressively to maintain heading.
     *
     * <p>This PID controller operates on heading radians and outputs a target rotational rate in radians per second.
     *
     * @param kP The proportional coefficient; must be >= 0
     * @param kI The integral coefficient; must be >= 0
     * @param kD The differential coefficient; must be >= 0
     * @return this object
     */
    public DriveToPose withHeadingPID(double kP, double kI, double kD) {
        this.thetaController.setPID(kP, kI, kD);
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     *
     * <p>The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public DriveToPose withTranslationDeadband(LinearVelocity newDeadband) {
        this.Deadband = newDeadband.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     *
     * <p>The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public DriveToPose withHeadingDeadband(AngularVelocity newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
        return this;
    }

    public DriveToPose withDistanceTolerance(double distanceTolerance) {
        this.distanceTolerance = distanceTolerance;
        return this;
    }

    public void resetNextLoop() {
        toReset = true;
    }
}
