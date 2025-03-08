package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static org.supurdueper.robot2025.Constants.DriverConstants.leftAutoAlighOffset;
import static org.supurdueper.robot2025.Constants.DriverConstants.rightAutoAlignOffset;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.Collections;
import java.util.Comparator;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.Constants.DriveConstants;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.Driver;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class FullAutoAim implements SwerveRequest {

    private RobotCentricFacingAngle robotCentricFacingAngle;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final PIDController mPathYController =
            new PIDController(DriveConstants.translationKp, DriveConstants.translationKi, DriveConstants.translationKd);
    private Driver driver;
    Pole pole;

    public enum Pole {
        LEFT,
        RIGHT
    }

    public FullAutoAim(Pole pole) {
        this.pole = pole;
        robotCentricFacingAngle = new RobotCentricFacingAngle();
        robotCentricFacingAngle.HeadingController.setPID(
                DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
        robotCentricFacingAngle.HeadingController.setTolerance(Degrees.of(2).in(Radians));
        robotCentricFacingAngle.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
        this.driver = RobotContainer.getDriver();
        mPathYController.setTolerance(Inch.of(1).in(Meters));
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {

        // Set rotation based on what side of the reef we are facing. Also grab the
        // april tag in view
        Translation2d reefCenter = AllianceFlip.apply(FieldConstants.Reef.center);
        Rotation2d facingReefCenter =
                reefCenter.minus(parameters.currentPose.getTranslation()).getAngle();
        robotCentricFacingAngle.TargetDirection = Collections.min(
                FieldConstants.reefAngles, Comparator.comparing(angle -> absDistanceRadians(angle, facingReefCenter)));

        int aprilTagId = FieldConstants.getClosestReefTagId(robotCentricFacingAngle.TargetDirection);

        // PID
        Pose2d currentPoseFacingReef =
                new Pose2d(parameters.currentPose.getTranslation(), robotCentricFacingAngle.TargetDirection);

        Distance yOffset = pole == Pole.LEFT ? leftAutoAlighOffset : rightAutoAlignOffset;
        if (aprilTagId == 10 || aprilTagId == 21) {
            yOffset = pole == Pole.RIGHT ? leftAutoAlighOffset : rightAutoAlignOffset;
        }
        double error =
                FieldConstants.getRobotPoseTargetSpace(currentPoseFacingReef).getY() + yOffset.in(Meters);
        robotCentricFacingAngle.VelocityY = mPathYController.calculate(error);
        double throttle = getFieldCentricJoystick(
                driver.getDriveFwdPositive(),
                driver.getDriveLeftPositive(),
                AllianceFlip.apply(parameters.currentPose.getRotation()));
        robotCentricFacingAngle.VelocityX = throttle * MaxSpeed;
        return robotCentricFacingAngle.apply(parameters, modulesToApply);
    }

    private double absDistanceRadians(Rotation2d angle1, Rotation2d angle2) {
        return Math.abs(angle1.minus(angle2).getRadians());
    }

    private double getFieldCentricJoystick(double fwdPositive, double leftPositive, Rotation2d robotAngle) {
        Translation2d joystickPos = new Translation2d(leftPositive, fwdPositive);
        joystickPos = joystickPos.rotateBy(robotAngle);
        return joystickPos.getY();
    }
}
