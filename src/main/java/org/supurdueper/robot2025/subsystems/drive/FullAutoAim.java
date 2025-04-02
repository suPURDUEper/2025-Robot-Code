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
import org.supurdueper.lib.utils.GeomUtil;
import org.supurdueper.robot2025.Constants.DriveConstants;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.state.RobotStates;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class FullAutoAim implements SwerveRequest {

    private FieldCentricFacingAngle swerveRequest;
    private final PhoenixProfiledPIDController throttleController = new PhoenixProfiledPIDController(
            DriveConstants.translationKp,
            DriveConstants.translationKi,
            DriveConstants.translationKd,
            new TrapezoidProfile.Constraints(
                    TunerConstants.kMaxAutoAimSpeed.in(MetersPerSecond),
                    TunerConstants.kMaxAutoAimAcceleration.in(MetersPerSecondPerSecond)));
    Pole pole;

    Translation2d lastSetpointTranslation;

    public enum Pole {
        LEFT,
        RIGHT
    }

    private final Transform2d leftRobotScoringOffset =
            new Transform2d(DriveConstants.robotToBumperCenter, DriveConstants.leftAutoAlignOffset, Rotation2d.k180deg);
    private final Transform2d rightRobotScoringOffset = new Transform2d(
            DriveConstants.robotToBumperCenter, DriveConstants.rightAutoAlignOffset, Rotation2d.k180deg);
    private boolean reset = true;

    public FullAutoAim(Pole pole) {

        this.pole = pole;
        swerveRequest = new FieldCentricFacingAngle();
        swerveRequest.HeadingController.setPID(
                DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
        swerveRequest.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
        swerveRequest.Deadband = translationClosedLoopDeadband.in(MetersPerSecond);
        swerveRequest.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);
    }

    public void reset(SwerveControlParameters parameters, Pose2d goalPose) {
        // Turn chassis speeds into field speeds
        if (reset) {
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
        // Set rotation based on what side of the reef we are facing. Also grab the
        // april tag in view
        swerveRequest.TargetDirection = FieldConstants.getClosestReefAngle(parameters.currentPose.getTranslation());

        // Calculate goal pose
        Transform2d offset = pole == Pole.LEFT ? leftRobotScoringOffset : rightRobotScoringOffset;
        Pose2d aprilTagPose =
                FieldConstants.getAprilTagPose(FieldConstants.getReefTagId(swerveRequest.TargetDirection));
        Pose2d goalPose = aprilTagPose.plus(offset);
        DogLog.log("Auto Aim/Goal Position", goalPose);

        if (RobotStates.atL1.getAsBoolean()) {
            swerveRequest.TargetDirection = swerveRequest.TargetDirection.rotateBy(Rotation2d.k180deg);
        }

        reset(parameters, goalPose);

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
