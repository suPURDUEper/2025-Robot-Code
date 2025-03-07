package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units.*;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.Driver;
import org.supurdueper.robot2025.subsystems.Vision;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class FullAutoAim implements SwerveRequest {

    private RobotCentricFacingAngle robotCentricFacingAngle;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final PIDController m_pathXController = new PIDController(5, 0, 0);

    Translation2d reefCenterBlue = FieldConstants.Reef.center;
    final List<Rotation2d> reefAngles = Arrays.asList(new Rotation2d[] {
        Rotation2d.kZero,
        new Rotation2d(Math.PI / 3),
        new Rotation2d(2 * Math.PI / 3),
        Rotation2d.kPi,
        new Rotation2d(4 * Math.PI / 3),
        new Rotation2d(5 * Math.PI / 3)
    });

    final Map<Rotation2d, Integer> reefAngleToAprilTagIdBlue;
    final Map<Rotation2d, Integer> reefAngleToAprilTagIdRed;

    private Driver driver;
    private String limelightName;

    public FullAutoAim(String limelightName) {
        this.limelightName = limelightName;
        robotCentricFacingAngle = new RobotCentricFacingAngle();
        robotCentricFacingAngle.HeadingController.setPID(5, 0, .75);
        robotCentricFacingAngle.HeadingController.setTolerance(Degrees.of(1).in(Radians));
        robotCentricFacingAngle.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
        reefAngleToAprilTagIdBlue = Map.of(
                reefAngles.get(0), 18,
                reefAngles.get(1), 17,
                reefAngles.get(2), 22,
                reefAngles.get(3), 21,
                reefAngles.get(4), 29,
                reefAngles.get(5), 19);

        reefAngleToAprilTagIdRed = Map.of(
                reefAngles.get(0), 10,
                reefAngles.get(1), 11,
                reefAngles.get(2), 6,
                reefAngles.get(3), 7,
                reefAngles.get(4), 8,
                reefAngles.get(5), 9);
        this.driver = RobotContainer.getDriver();
        m_pathXController.setTolerance(Inch.of(1).in(Meters));
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {

        // Set rotation based on what side of the reef we are facing. Also grab the
        // april tag in view
        Translation2d reefCenter = AllianceFlip.apply(reefCenterBlue);
        Rotation2d facingReefCenter =
                reefCenter.minus(parameters.currentPose.getTranslation()).getAngle();
        robotCentricFacingAngle.TargetDirection =
                Collections.min(reefAngles, Comparator.comparing(angle -> absDistanceRadians(angle, facingReefCenter)));

        // PID
        double error = Vision.getHorizonalOffsetFromTargetMeters(limelightName);
        robotCentricFacingAngle.VelocityY = m_pathXController.calculate(error);
        robotCentricFacingAngle.VelocityX = driver.getDriveFwdPositive() * MaxSpeed;
        return robotCentricFacingAngle.apply(parameters, modulesToApply);
    }

    private double absDistanceRadians(Rotation2d angle1, Rotation2d angle2) {
        return Math.abs(angle1.minus(angle2).getRadians());
    }
}
