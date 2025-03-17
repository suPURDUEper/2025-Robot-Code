package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2025.Constants.DriveConstants.*;
import static org.supurdueper.robot2025.state.RobotStates.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.supurdueper.lib.swerve.DriveToPose;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.Constants.DriveConstants;
import org.supurdueper.robot2025.Constants.DriveConstants.*;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.Driver;
import org.supurdueper.robot2025.state.RobotStates;
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
    private final FieldCentricFacingReef fieldCentricFacingReef = new FieldCentricFacingReef();
    private final FieldCentricFacingAngle fieldCentricFacingAngle = new FieldCentricFacingAngle();
    private final FieldCentricFacingAngle fieldCentricFacingHpStation = new FieldCentricFacingHpStation();
    private final DriveToPose driveToPose = new DriveToPose();
    private final FullAutoAim leftAim = new FullAutoAim(Pole.LEFT);
    private final FullAutoAim rightAim = new FullAutoAim(Pole.RIGHT);
    private final Transform2d leftRobotScoringOffset =
            new Transform2d(DriveConstants.robotToBumperCenter, DriveConstants.leftAutoAlighOffset, Rotation2d.k180deg);
    private final Transform2d rightRobotScoringOffset = new Transform2d(
            DriveConstants.robotToBumperCenter, DriveConstants.rightAutoAlignOffset, Rotation2d.k180deg);

    public DriveStates(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.driver = RobotContainer.getDriver();
        fieldCentricFacingAngle.HeadingController.setPID(headingKp, headingKi, headingKd);
        fieldCentricFacingAngle.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);

        driveToPose
                .withTranslationPID(translationKp, translationKi, translationKd)
                .withTranslationConstraints(TunerConstants.kMaxAutoAimSpeed, TunerConstants.kMaxAcceleration)
                .withTranslationDeadband(translationClosedLoopDeadband)
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
        actionLeftAim.whileTrue(leftAlign());
        actionRightAim.whileTrue(rightAlign());
    }

    private Command normalTeleopDrive() {
        return drivetrain.applyRequest(() -> driveFieldCentric
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withRotationalRate(driver.getDriveCCWPositive() * MaxAngularRate));
    }

    private Command driveFacingReef() {
        return drivetrain.applyRequest(() -> fieldCentricFacingReef
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed));
    }

    private Command driveFacingHpStation() {
        return drivetrain.applyRequest(() -> fieldCentricFacingHpStation
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed));
    }

    private Command driveFacingAngle(Rotation2d angle) {
        return drivetrain.applyRequest(() -> fieldCentricFacingAngle
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withTargetDirection(angle));
    }

    private Command driveFacingProcessor() {
        return driveFacingAngle(Rotation2d.kCW_90deg);
    }

    private Command driveFacingNet() {
        return driveFacingAngle(Rotation2d.kZero);
    }

    private Command leftAlign() {
        return drivetrain.applyRequest(() -> leftAim);
    }

    private Command align(Transform2d aprilTagScoringOffset) {
        return drivetrain.applyRequest(() -> {
            Pose2d currentRobotPose = drivetrain.getState().Pose;
            int apriltagId = FieldConstants.getClosestReefTagId(currentRobotPose);
            Pose2d targetPose = FieldConstants.getAprilTagPose(apriltagId).plus(aprilTagScoringOffset);
            return driveToPose.withGoal(targetPose);
        });
    }

    private Command rightAlign() {
        return drivetrain.applyRequest(() -> rightAim);
    }
}
