package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2025.state.RobotStates.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.Driver;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class DriveStates {

    private Drivetrain drivetrain;
    private Driver driver;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private final FieldCentricFacingReef fieldCentricFacingReef = new FieldCentricFacingReef();
    private final FieldCentricFacingAngle fieldCentricFacingAngle = new FieldCentricFacingAngle();

    public DriveStates() {
        this.drivetrain = RobotContainer.getDrivetrain();
        this.driver = RobotContainer.getDriver();
    }

    public void bindCommands() {
        drivetrain.setDefaultCommand(normalTeleopDrive());
        actionReef.onTrue(driveFacingReef());
        actionNet.onTrue(driveFacingNet());
        actionProcessor.onTrue(driveFacingProcessor());
        actionIntake.or(actionHome, actionClimbPrep, actionScore).onTrue(normalTeleopDrive());
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

    private Command driveFacingAngle(Supplier<Rotation2d> angle) {
        return drivetrain.applyRequest(() -> fieldCentricFacingAngle
                .withVelocityX(driver.getDriveFwdPositive() * MaxSpeed)
                .withVelocityY(driver.getDriveLeftPositive() * MaxSpeed)
                .withTargetDirection(angle.get()));
    }

    private Command driveFacingProcessor() {
        return driveFacingAngle(() -> Rotation2d.kCCW_90deg);
    }

    private Command driveFacingNet() {
        return driveFacingAngle(() -> Rotation2d.k180deg);
    }
}
