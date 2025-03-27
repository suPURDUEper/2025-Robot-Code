package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2025.Constants.DriveConstants.*;
import static org.supurdueper.robot2025.state.RobotStates.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.supurdueper.lib.swerve.DriveToPose;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.Constants.DriveConstants.*;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.Driver;
import org.supurdueper.robot2025.state.RobotStates;
import org.supurdueper.robot2025.subsystems.drive.DriveSysId.SysIdSwerveTranslationCurrent;
import org.supurdueper.robot2025.subsystems.drive.FullAutoAim.Pole;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class DriveStates {

    private Drivetrain drivetrain;
    private Driver driver;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private double MaxSpeed = TunerConstants.kMaxSpeed.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();
    private final SysIdSwerveTranslationCurrent driveCurrentTuning = new SysIdSwerveTranslationCurrent();
    private final FieldCentricFacingReef fieldCentricFacingReef = new FieldCentricFacingReef();
    private final FieldCentricFacingAngle fieldCentricFacingAngle = new FieldCentricFacingAngle();
    private final FieldCentricFacingAngle fieldCentricFacingHpStation = new FieldCentricFacingHpStation();
    private final DriveToPose driveToPose = new DriveToPose();
    private final FullAutoAim leftAim = new FullAutoAim(Pole.LEFT);
    private final FullAutoAim rightAim = new FullAutoAim(Pole.RIGHT);

    public DriveStates(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.driver = RobotContainer.getDriver();
        fieldCentricFacingAngle.HeadingController.setPID(headingKp, headingKi, headingKd);
        fieldCentricFacingAngle.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);

        driveToPose
                .withTranslationPID(translationKp, translationKi, translationKd)
                .withTranslationConstraints(TunerConstants.kMaxAutoAimSpeed, TunerConstants.kMaxAutoAimAcceleration)
                .withTranslationDeadband(translationClosedLoopDeadband)
                .withDistanceTolerance(positionTolerance.in(Meters))
                .withHeadingPID(headingKp, headingKi, headingKd)
                .withHeadingConstraints(RotationsPerSecond.of(0.75), RotationsPerSecondPerSecond.of(1.5))
                .withHeadingDeadband(rotationClosedLoopDeadband);
    }

    public void bindCommands() {
        drivetrain.setDefaultCommand(normalTeleopDrive());
        atReefNoL1.and(RobotStates.teleop).whileTrue(driveFacingReef());
        atIntake.and(RobotStates.hasCoral.negate()).and(RobotStates.teleop).whileTrue(driveFacingHpStation());

        atProcessor.and(RobotStates.teleop).whileTrue(driveFacingProcessor());
        atNet.and(RobotStates.teleop).whileTrue(driveFacingNet());
        actionClimbPrep.onTrue(normalTeleopDrive());
        rezeroFieldHeading.onTrue(
                Commands.runOnce(() -> drivetrain.resetRotation(AllianceFlip.apply(Rotation2d.kZero))));
        // actionLeftAim.whileTrue(align(leftRobotScoringOffset));
        // actionRightAim.whileTrue(align(rightRobotScoringOffset));
        actionLeftAim.whileTrue(leftAlign());
        actionRightAim.whileTrue(rightAlign());
    }

    private Command normalTeleopDrive() {
        return drivetrain.applyRequest(() -> driveFieldCentric
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withRotationalRate(driver.getDriveCCWPositive() * MaxAngularRate)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    public Command setVelocityDrive(double velocity) {
        return drivetrain.applyRequest(
                () -> driveFieldCentric.withVelocityX(velocity).withDriveRequestType(DriveRequestType.Velocity));
    }

    private Command currentTuningDrive() {
        return drivetrain.applyRequest(() -> driveCurrentTuning.withCurrent(driver.getDriveFwdPositive() * 40));
    }

    public Command driveToPose(Supplier<Pose2d> targetSupplier) {
        Pose2d targetPose = targetSupplier.get();
        return drivetrain.applyRequest(() -> driveToPose.withGoal(targetPose));
    }

    private Command driveFacingReef() {
        return drivetrain.applyRequest(() -> fieldCentricFacingReef
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    private Command driveFacingHpStation() {
        return drivetrain.applyRequest(() -> fieldCentricFacingHpStation
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    private Command driveFacingAngle(Rotation2d angle) {
        return drivetrain.applyRequest(() -> fieldCentricFacingAngle
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withTargetDirection(angle)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    private Command driveFacingProcessor() {
        return driveFacingAngle(Rotation2d.kCW_90deg);
    }

    private Command driveFacingNet() {
        return driveFacingAngle(Rotation2d.kZero);
    }

    private Command leftAlign() {
        return Commands.runOnce(() -> {
                    RobotStates.setAimed(false);
                    SwerveDriveState state = drivetrain.getState();
                    Pose2d goalPose = FieldConstants.getAprilTagPose(FieldConstants.getReefTagId(
                            FieldConstants.getClosestReefAngle(state.Pose.getTranslation())));
                    leftAim.setResetNextLoop();
                })
                .andThen(drivetrain.applyRequest(() -> leftAim));
    }

    private Command align(Transform2d aprilTagScoringOffset) {
        return Commands.runOnce(() -> driveToPose.resetNextLoop()).andThen(drivetrain.applyRequest(() -> {
            Pose2d currentRobotPose = drivetrain.getState().Pose;
            int apriltagId = FieldConstants.getClosestReefTagId(currentRobotPose);
            Pose2d targetPose = FieldConstants.getAprilTagPose(apriltagId).plus(aprilTagScoringOffset);
            return driveToPose.withGoal(targetPose);
        }));
    }

    private Command rightAlign() {
        return Commands.runOnce(() -> {
                    RobotStates.setAimed(false);
                    SwerveDriveState state = drivetrain.getState();
                    Pose2d goalPose = FieldConstants.getAprilTagPose(FieldConstants.getReefTagId(
                            FieldConstants.getClosestReefAngle(state.Pose.getTranslation())));
                    rightAim.setResetNextLoop();
                    ;
                })
                .andThen(drivetrain.applyRequest(() -> rightAim));
    }

    private double absDistanceRadians(Rotation2d angle1, Rotation2d angle2) {
        return Math.abs(angle1.minus(angle2).getRadians());
    }
}
