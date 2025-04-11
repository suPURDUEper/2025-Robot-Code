package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2025.Constants.DriveConstants.*;
import static org.supurdueper.robot2025.state.RobotStates.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.supurdueper.lib.swerve.DriveToPose;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.Constants.DriveConstants.*;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.Driver;
import org.supurdueper.robot2025.state.RobotStates;
import org.supurdueper.robot2025.subsystems.drive.CoralAutoAim.Pole;
import org.supurdueper.robot2025.subsystems.drive.DriveSysId.SysIdSwerveTranslationCurrent;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class DriveStates {

    private Drivetrain drivetrain;
    private Driver driver;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private double MaxSpeed = TunerConstants.kMaxSpeed.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private final SysIdSwerveTranslationCurrent driveCurrentTuning = new SysIdSwerveTranslationCurrent();
    private final FieldCentricFacingReef fieldCentricFacingReef = new FieldCentricFacingReef();
    private final FieldCentricFacingReefL1 fieldCentricFacingReefL1 = new FieldCentricFacingReefL1();
    private final FieldCentricFacingAngle fieldCentricFacingAngle = new FieldCentricFacingAngle();
    private final FieldCentricFacingAngle fieldCentricFacingHpStation = new FieldCentricFacingHpStation();
    private final DriveToPose driveToPose;
    private final CoralAutoAim leftAim = new CoralAutoAim(Pole.LEFT);
    private final CoralAutoAim rightAim = new CoralAutoAim(Pole.RIGHT);
    private final BargeAutoAim bargeAutoAim = new BargeAutoAim();
    private final L1LoadAutoAim l1LoadAutoAim = new L1LoadAutoAim();
    private final LollipopAutoAim lollipopAutoAim = new LollipopAutoAim();

    public DriveStates(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.driver = RobotContainer.getDriver();
        fieldCentricFacingAngle.HeadingController.setPID(headingKp, headingKi, headingKd);
        fieldCentricFacingAngle.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);

        driveToPose = new DriveToPose()
                .withDrivePID(translationKp, translationKi, translationKd)
                .withDriveConsraints(TunerConstants.kMaxAutoAimSpeed, TunerConstants.kMaxAutoAimAcceleration)
                .withDriveDeadband(translationClosedLoopDeadband)
                .withPositionTolerance(positionTolerance)
                .withHeadingPID(headingKp, headingKi, headingKd)
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
        actionLeftAim.and(RobotStates.atReefNoL1).whileTrue(reefLeftPoleAlign());
        actionRightAim.and(RobotStates.atReefNoL1).whileTrue(reefRightPoleAlign());
        actionAim.and(RobotStates.atNet).whileTrue(bargeAlign());
        RobotStates.atL1.whileTrue(driveFacingHpStation());
        RobotStates.actionL1.onTrue(driveFacingHpStation());
        RobotStates.atL1.and(RobotStates.actionAim).whileTrue(l1LoadAutoAim());
        RobotStates.actionLollipop.whileTrue(lollipopAutoAim());
        actionScore.and(RobotStates.atL1).whileTrue(driveFacingReefL1());
        actionScore.onFalse(normalTeleopDrive());
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

    private Command driveFacingReefL1() {
        return drivetrain.applyRequest(() -> fieldCentricFacingReefL1
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

    private Command reefLeftPoleAlign() {
        return Commands.runOnce(() -> {
                    RobotStates.setAimed(false);
                    leftAim.setResetNextLoop();
                })
                .andThen(drivetrain.applyRequest(() -> leftAim));
    }

    private Command newLeftAlign() {
        return new AutoAim(drivetrain, true, () -> driveToPose);
    }

    private Command newRightAlign() {
        return new AutoAim(drivetrain, false, () -> driveToPose);
    }

    private Command reefRightPoleAlign() {
        return Commands.runOnce(() -> {
                    RobotStates.setAimed(false);
                    rightAim.setResetNextLoop();
                })
                .andThen(drivetrain.applyRequest(() -> rightAim));
    }

    private Command bargeAlign() {
        return Commands.runOnce(() -> {
                    RobotStates.setAimed(false);
                    bargeAutoAim.setResetNextLoop();
                })
                .andThen(drivetrain.applyRequest(() -> bargeAutoAim));
    }

    private Command l1LoadAutoAim() {
        return Commands.runOnce(() -> {
                    l1LoadAutoAim.setResetNextLoop();
                })
                .andThen(drivetrain.applyRequest(() -> l1LoadAutoAim));
    }

    private Command lollipopAutoAim() {
        return Commands.runOnce(() -> {
                    lollipopAutoAim.setResetNextLoop();
                })
                .andThen(drivetrain.applyRequest(() -> lollipopAutoAim));
    }
}
