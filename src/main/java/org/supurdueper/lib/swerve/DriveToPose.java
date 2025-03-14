package org.supurdueper.lib.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib convention, so this
     * determines how fast to travel forward.
     */
    public double VelocityX = 0;
    /**
     * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib convention, so this
     * determines how fast to travel to the left.
     */
    public double VelocityY = 0;
    /**
     * The desired direction to face. 0 Degrees is defined as in the direction of the X axis. As a result, a
     * TargetDirection of 90 degrees will point along the Y axis, or to the left.
     */
    public Rotation2d TargetDirection = new Rotation2d();
    /**
     * The rotational rate feedforward to add to the output of the heading controller, in radians per second. When using
     * a motion profile for the target direction, this can be set to the current velocity reference of the profile.
     */
    public double rotationFeedforward = 0;

    /**
     * The rotational rate feedforward to add to the output of the heading controller, in radians per second. When using
     * a motion profile for the target direction, this can be set to the current velocity reference of the profile.
     */
    public double xFeedforward = 0;

    /**
     * The rotational rate feedforward to add to the output of the heading controller, in radians per second. When using
     * a motion profile for the target direction, this can be set to the current velocity reference of the profile.
     */
    public double yFeedforward = 0;

    /** The allowable deadband of the request, in m/s. */
    public double Deadband = 0;
    /** The rotational deadband of the request, in radians per second. */
    public double RotationalDeadband = 0;

    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will rotate around the
     * center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

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
    private PhoenixProfiledPIDController yController;
    private PhoenixProfiledPIDController xController;
    private PhoenixProfiledPIDController thetaController;
    private Pose2d goal;

    public DriveToPose() {
        this.goal = Pose2d.kZero;
        fieldCentric = new FieldCentric();
        yController = new PhoenixProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        xController = new PhoenixProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        thetaController = new PhoenixProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        // Handle x
        double xGoal = goal.getTranslation().getX();
        VelocityX =
                xController.calculate(parameters.currentPose.getTranslation().getX(), xGoal, parameters.timestamp);
        xFeedforward = xController.getSetpoint().velocity;
        VelocityX += xFeedforward;

        // Handle y
        double yGoal = goal.getTranslation().getY();
        VelocityY =
                xController.calculate(parameters.currentPose.getTranslation().getY(), yGoal, parameters.timestamp);
        yFeedforward = yController.getSetpoint().velocity;
        VelocityX += yFeedforward;

        // Handle rotation
        Rotation2d angleToFace = goal.getRotation();
        if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            /*
             * If we're operator perspective, rotate the direction we want to face by the
             * angle
             */
            angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
        }
        double toApplyOmega = thetaController.calculate(
                parameters.currentPose.getRotation().getRadians(), angleToFace.getRadians(), parameters.timestamp);
        rotationFeedforward = thetaController.getSetpoint().velocity;
        toApplyOmega += rotationFeedforward;

        return fieldCentric
                .withVelocityX(VelocityX)
                .withVelocityY(VelocityY)
                .withRotationalRate(toApplyOmega)
                .withDeadband(Deadband)
                .withRotationalDeadband(RotationalDeadband)
                .withCenterOfRotation(CenterOfRotation)
                .withDriveRequestType(DriveRequestType)
                .withSteerRequestType(SteerRequestType)
                .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
                .withForwardPerspective(ForwardPerspective)
                .apply(parameters, modulesToApply);
    }

    public DriveToPose withGoal(Pose2d goal) {
        this.goal = goal;
        return this;
    }

    public DriveToPose withTranslationConstraints(LinearVelocity maxSpeed, LinearAcceleration maxAcceleration) {
        yController.setConstraints(new TrapezoidProfile.Constraints(
                maxSpeed.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond)));
        xController.setConstraints(new TrapezoidProfile.Constraints(
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
        this.xController.setPID(kP, kI, kD);
        this.yController.setPID(kP, kI, kD);
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
    public DriveToPose withRotationalDeadband(AngularVelocity newRotationalDeadband) {
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
