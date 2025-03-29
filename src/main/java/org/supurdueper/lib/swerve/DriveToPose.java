package org.supurdueper.lib.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import org.supurdueper.lib.utils.GeomUtil;
import org.supurdueper.robot2025.state.RobotStates;

public class DriveToPose implements SwerveRequest {

    private FieldCentricFacingAngle swerveRequest;
    private final PhoenixProfiledPIDController throttleController =
            new PhoenixProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    private Pose2d goalPose = Pose2d.kZero;
    Translation2d lastSetpointTranslation;
    Distance positionTolerance = Meters.of(0);
    private boolean reset = true;

    public DriveToPose() {
        swerveRequest = new FieldCentricFacingAngle();
        swerveRequest.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
    }

    public void reset(SwerveControlParameters parameters, Pose2d goalPose) {
        // Turn chassis speeds into field speeds
        if (reset) {
            ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                    parameters.currentChassisSpeed, parameters.currentPose.getRotation());
            Translation2d linearFieldVelocity =
                    new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
            double resetDistance = parameters.currentPose.getTranslation().getDistance(goalPose.getTranslation());
            DogLog.log("Drive To Pose/Reset Distance", resetDistance);
            throttleController.reset(
                    resetDistance,
                    Math.min(
                            0.0,
                            -linearFieldVelocity
                                    .rotateBy(goalPose.getTranslation()
                                            .minus(parameters.currentPose.getTranslation())
                                            .getAngle()
                                            .unaryMinus())
                                    .getX()),
                    parameters.timestamp);
            throttleController.setGoal(0);

            lastSetpointTranslation = parameters.currentPose.getTranslation();
        }
        reset = false;
    }

    public void resetNextLoop() {
        reset = true;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        reset(parameters, goalPose);

        // Calculate x and y velocities
        double distanceToGoalMeters = parameters.currentPose.getTranslation().getDistance(goalPose.getTranslation());
        double driveVelocityMagnitude = throttleController.calculate(distanceToGoalMeters, 0, parameters.timestamp);
        double ffMinRadius = 0.05;
        double ffMaxRadius = 1;
        double ffScaler =
                MathUtil.clamp((driveVelocityMagnitude - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
        driveVelocityMagnitude += throttleController.getSetpoint().velocity * ffScaler;
        // Check if we're aimed
        if (distanceToGoalMeters < positionTolerance.in(Meters)) {
            RobotStates.setAimed(true);
            driveVelocityMagnitude = 0;
        } else {
            RobotStates.setAimed(false);
        }
        DogLog.log("Drive To Pose/Distance To Goal", distanceToGoalMeters);
        DogLog.log("Drive To Pose/Throttle Setpoint", throttleController.getSetpoint().position);

        Translation2d driveVelocity = new Pose2d(
                        Translation2d.kZero,
                        new Rotation2d(Math.atan2(
                                parameters.currentPose.getTranslation().getY()
                                        - goalPose.getTranslation().getY(),
                                parameters.currentPose.getTranslation().getX()
                                        - goalPose.getTranslation().getX())))
                .transformBy(GeomUtil.toTransform2d(driveVelocityMagnitude, 0.0))
                .getTranslation();

        swerveRequest.VelocityY = driveVelocity.getY();
        swerveRequest.VelocityX = driveVelocity.getX();

        // Log setpoint
        Translation2d lastSetpointTranslation = new Pose2d(
                        goalPose.getTranslation(),
                        new Rotation2d(Math.atan2(
                                parameters.currentPose.getTranslation().getY()
                                        - goalPose.getTranslation().getY(),
                                parameters.currentPose.getTranslation().getX()
                                        - goalPose.getTranslation().getX())))
                .transformBy(GeomUtil.toTransform2d(throttleController.getSetpoint().position, 0.0))
                .getTranslation();
        Pose2d lastSetpoint = new Pose2d(lastSetpointTranslation, swerveRequest.TargetDirection);
        DogLog.log("Drive To Pose/Setpoint Position", lastSetpoint);

        return swerveRequest.withDriveRequestType(DriveRequestType.Velocity).apply(parameters, modulesToApply);
    }

    public DriveToPose withDrivePID(double kP, double kI, double kD) {
        throttleController.setPID(kP, kI, kD);
        return this;
    }

    public DriveToPose withDriveConsraints(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
        throttleController.setConstraints(new TrapezoidProfile.Constraints(
                maxVelocity.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond)));
        return this;
    }

    public DriveToPose withDriveDeadband(LinearVelocity deadband) {
        swerveRequest.Deadband = deadband.in(MetersPerSecond);
        return this;
    }

    public DriveToPose withHeadingPID(double kP, double kI, double kD) {
        swerveRequest.HeadingController.setPID(kP, kI, kD);
        return this;
    }

    public DriveToPose withHeadingDeadband(AngularVelocity deadband) {
        swerveRequest.RotationalDeadband = deadband.in(RadiansPerSecond);
        return this;
    }

    public DriveToPose withPositionTolerance(Distance tolerance) {
        positionTolerance = tolerance;
        return this;
    }

    public DriveToPose withGoal(Pose2d goal) {
        this.goalPose = goal;
        return this;
    }
}
