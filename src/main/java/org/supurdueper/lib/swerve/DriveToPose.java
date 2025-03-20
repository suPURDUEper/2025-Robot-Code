package org.supurdueper.lib.swerve;

import static edu.wpi.first.units.Units.*;

import org.supurdueper.lib.utils.GeomUtil;
import org.supurdueper.robot2025.Constants.DriveConstants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveToPose implements SwerveRequest {

    /** The allowable deadband of the request, in m/s. */
    public double Deadband = 0;
    /** The rotational deadband of the request, in radians per second. */
    public double RotationalDeadband = 0;

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    /**
     * Whether to desaturate wheel speeds before applying. For more information, see the documentation of
     * {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;

    /** The perspective to use when determining which direction is forward. */
    public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    private FieldCentric fieldCentric;
    private PhoenixProfiledPIDController driveController;
    private PhoenixProfiledPIDController thetaController;
    private Pose2d targetPose;
    Translation2d lastSetpointTranslation; 

    public DriveToPose() {
        this.targetPose = Pose2d.kZero;
        fieldCentric = new FieldCentric();
        driveController = new PhoenixProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        thetaController = new PhoenixProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        if (lastSetpointTranslation == null) {
            lastSetpointTranslation = parameters.currentPose.getTranslation();
        }
        double ffMinRadius = 0.05;
        double ffMaxRadius = 1;
        double currentDistanceError = parameters.currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler =
            MathUtil.clamp(
                (currentDistanceError - ffMinRadius / (ffMaxRadius - ffMinRadius)),
                0.0,
                1.0);

        driveController.reset(
            lastSetpointTranslation.getDistance(targetPose.getTranslation()),
            driveController.getSetpoint().velocity);
        double driveVelocityScalar =
            driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(currentDistanceError, 0.0, parameters.timestamp);
        lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                new Rotation2d(
                    Math.atan2(
                        parameters.currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        parameters.currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

        Translation2d driveVelocity =
        new Pose2d(
                Translation2d.kZero,
                new Rotation2d(
                    Math.atan2(
                        parameters.currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        parameters.currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

        // Handle rotation
        Rotation2d angleToFace = targetPose.getRotation();
        if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            /*
             * If we're operator perspective, rotate the direction we want to face by the
             * angle
             */
            angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
        }
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler + thetaController.calculate(
                parameters.currentPose.getRotation().getRadians(), angleToFace.getRadians(), parameters.timestamp);
        
        return fieldCentric
                .withVelocityX(driveVelocity.getX())
                .withVelocityY(driveVelocity.getY())
                .withRotationalRate(thetaVelocity)
                .withDeadband(Deadband)
                .withRotationalDeadband(RotationalDeadband)
                .withDriveRequestType(DriveRequestType)
                .withSteerRequestType(SteerRequestType)
                .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
                .withForwardPerspective(ForwardPerspective)
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

    /**
     * Modifies the ForwardPerspective parameter and returns itself.
     *
     * <p>The perspective to use when determining which direction is forward.
     *
     * @param newForwardPerspective Parameter to modify
     * @return this object
     */
    public DriveToPose withForwardPerspective(ForwardPerspectiveValue newForwardPerspective) {
        this.ForwardPerspective = newForwardPerspective;
        return this;
    }
}
