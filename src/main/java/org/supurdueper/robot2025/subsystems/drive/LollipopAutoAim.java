package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2025.Constants.DriveConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import org.supurdueper.lib.swerve.PhoenixProfiledPIDController;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.lib.utils.GeomUtil;
import org.supurdueper.robot2025.Constants.DriveConstants;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.state.RobotStates;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class LollipopAutoAim implements SwerveRequest {

    private FieldCentricFacingAngle swerveRequest;
    private final PhoenixProfiledPIDController throttleController = new PhoenixProfiledPIDController(
            DriveConstants.translationKp,
            DriveConstants.translationKi,
            DriveConstants.translationKd,
            new TrapezoidProfile.Constraints(1, TunerConstants.kMaxAutoAimAcceleration.in(MetersPerSecondPerSecond)));
    Translation2d lastSetpointTranslation;

    private boolean reset = true;
    Pose2d goalPose;

    public LollipopAutoAim() {
        swerveRequest = new FieldCentricFacingAngle();
        swerveRequest.HeadingController.setPID(
                DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
        swerveRequest.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
        swerveRequest.Deadband = translationClosedLoopDeadband.in(MetersPerSecond);
        swerveRequest.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);
    }

    public void reset(SwerveControlParameters parameters) {
        // Turn chassis speeds into field speeds
        if (reset) {
            Translation2d goalTranslation = AllianceFlip.apply(FieldConstants.StagingPositions.leftIceCream)
                    .getTranslation();
            Rotation2d goalRotation = parameters
                    .currentPose
                    .getTranslation()
                    .minus(goalTranslation)
                    .getAngle()
                    .rotateBy(Rotation2d.k180deg);
            goalPose = new Pose2d(goalTranslation, goalRotation);
            Transform2d offset = new Transform2d(0, Inches.of(4).in(Meters), goalRotation);
            goalPose = goalPose.plus(offset);
            goalPose = new Pose2d(goalPose.getTranslation(), goalRotation);
            DogLog.log("Lollipop Auto Aim/Goal Position", goalPose);
            ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                    parameters.currentChassisSpeed, parameters.currentPose.getRotation());
            Translation2d linearFieldVelocity =
                    new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
            double resetDistance = parameters.currentPose.getTranslation().getDistance(goalPose.getTranslation());
            DogLog.log("Auto Aim/Reset Distance", resetDistance);
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

    public void setResetNextLoop() {
        reset = true;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Distance positionTolerance = Inches.of(1);

        reset(parameters);

        // Calculate x and y velocities
        double distanceToGoalMeters = parameters.currentPose.getTranslation().getDistance(goalPose.getTranslation());
        double driveVelocityMagnitude = throttleController.calculate(distanceToGoalMeters, 0, parameters.timestamp);
        double ffMinRadius = 0.05;
        double ffMaxRadius = 1;
        double ffScaler = MathUtil.clamp((distanceToGoalMeters - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
        driveVelocityMagnitude += throttleController.getSetpoint().velocity * ffScaler;
        // Check if we're aimed
        if (distanceToGoalMeters < positionTolerance.in(Meters)) {
            RobotStates.setAimed(true);
            driveVelocityMagnitude = 0;
        } else {
            RobotStates.setAimed(false);
        }
        DogLog.log("Auto Aim/Distance To Goal", distanceToGoalMeters);
        DogLog.log("Auto Aim/Throttle Setpoint", throttleController.getSetpoint().position);

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
        swerveRequest.TargetDirection = goalPose.getRotation();

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
        DogLog.log("Auto Aim/Setpoint Position", lastSetpoint);

        return swerveRequest.withDriveRequestType(DriveRequestType.Velocity).apply(parameters, modulesToApply);
    }
}
