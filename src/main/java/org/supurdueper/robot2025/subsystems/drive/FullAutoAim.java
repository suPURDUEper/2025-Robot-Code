package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2025.Constants.DriveConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import java.util.Collections;
import java.util.Comparator;
import org.supurdueper.lib.swerve.PhoenixProfiledPIDController;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.Constants.DriveConstants;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.state.RobotStates;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class FullAutoAim implements SwerveRequest {

    private RobotCentricFacingAngle swerveRequest;
    private final PhoenixProfiledPIDController yController = new PhoenixProfiledPIDController(
            DriveConstants.translationKp,
            DriveConstants.translationKi,
            0.0,
            new TrapezoidProfile.Constraints(
                    TunerConstants.kMaxAutoAimSpeed.in(MetersPerSecond),
                    TunerConstants.kMaxAutoAimAcceleration.in(MetersPerSecondPerSecond)));
    private final PhoenixProfiledPIDController xController = new PhoenixProfiledPIDController(
            DriveConstants.translationKp,
            DriveConstants.translationKi,
            DriveConstants.translationKd,
            new TrapezoidProfile.Constraints(
                    TunerConstants.kMaxAutoAimSpeed.in(MetersPerSecond),
                    TunerConstants.kMaxAutoAimAcceleration.in(MetersPerSecondPerSecond)));
    private final PhoenixPIDController throttleController = new PhoenixPIDController(
            DriveConstants.translationKp, DriveConstants.translationKi, DriveConstants.translationKd);
    Pole pole;

    public enum Pole {
        LEFT,
        RIGHT
    }

    private final Transform2d leftRobotScoringOffset = new Transform2d(
            DriveConstants.robotToBumperCenter.unaryMinus(), DriveConstants.leftAutoAlighOffset, Rotation2d.k180deg);
    private final Transform2d rightRobotScoringOffset = new Transform2d(
            DriveConstants.robotToBumperCenter.unaryMinus(), DriveConstants.rightAutoAlignOffset, Rotation2d.k180deg);

    public FullAutoAim(Pole pole) {
        this.pole = pole;
        swerveRequest = new RobotCentricFacingAngle();
        swerveRequest.HeadingController.setPID(
                DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
        swerveRequest.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
        swerveRequest.Deadband = translationClosedLoopDeadband.in(MetersPerSecond);
        swerveRequest.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Distance positionTolerance = Inches.of(0.7);
        // Set rotation based on what side of the reef we are facing. Also grab the
        // april tag in view
        Translation2d reefCenter = AllianceFlip.apply(FieldConstants.Reef.center);
        Rotation2d facingReefCenter =
                reefCenter.minus(parameters.currentPose.getTranslation()).getAngle();
        swerveRequest.TargetDirection = Collections.min(
                FieldConstants.reefAngles, Comparator.comparing(angle -> absDistanceRadians(angle, facingReefCenter)));
        int aprilTagId = FieldConstants.getClosestReefTagId(swerveRequest.TargetDirection);

        // PID to specified left/right offset from apriltag
        Pose2d currentPoseFacingReef =
                new Pose2d(parameters.currentPose.getTranslation(), swerveRequest.TargetDirection);

        // Distance yOffset = pole == Pole.LEFT ? leftAutoAlighOffset : rightAutoAlignOffset;
        // Flip backside of reef based on driver preference
        // if (aprilTagId == 10 || aprilTagId == 21) {
        //     yOffset = pole == Pole.RIGHT ? leftAutoAlighOffset : rightAutoAlignOffset;
        // }
        // Distance xOffset = DriveConstants.robotToBumperCenter;
        Transform2d offset = pole == Pole.LEFT ? leftRobotScoringOffset : rightRobotScoringOffset;
        Pose2d robotPoseTargetSpace = FieldConstants.getRobotPoseTargetSpace(currentPoseFacingReef);
        Pose2d goalPoseTargetSpace = new Pose2d(offset.getX(), offset.getY(), swerveRequest.TargetDirection);
        swerveRequest.VelocityY =
                yController.calculate(robotPoseTargetSpace.getY(), goalPoseTargetSpace.getY(), parameters.timestamp);
        swerveRequest.VelocityY += yController.getSetpoint().velocity;
        swerveRequest.VelocityX =
                xController.calculate(robotPoseTargetSpace.getX(), goalPoseTargetSpace.getX(), parameters.timestamp);
        swerveRequest.VelocityX += xController.getSetpoint().velocity;
        double yError = goalPoseTargetSpace.getY() - robotPoseTargetSpace.getY();
        double xError = goalPoseTargetSpace.getX() - robotPoseTargetSpace.getX();
        if (Math.hypot(xError, yError) < positionTolerance.in(Meters)) {
            RobotStates.setAimed(true);
            swerveRequest.VelocityX = 0;
            swerveRequest.VelocityY = 0;
        } else {
            RobotStates.setAimed(false);
        }
        DogLog.log("Auto Aim Robot Pose", robotPoseTargetSpace);
        DogLog.log(
                "Auto Aim/Goal Position",
                new Translation2d(xController.getSetpoint().position, yController.getSetpoint().position));
        return swerveRequest.withDriveRequestType(DriveRequestType.Velocity).apply(parameters, modulesToApply);
    }

    private double absDistanceRadians(Rotation2d angle1, Rotation2d angle2) {
        return Math.abs(angle1.minus(angle2).getRadians());
    }

    // private double getFieldCentricJoystick(double fwdPositive, double leftPositive, Rotation2d robotAngle) {
    //     Translation2d joystickPos = new Translation2d(leftPositive, fwdPositive);
    //     joystickPos = joystickPos.rotateBy(robotAngle);
    //     return joystickPos.getY();
    // }
}
