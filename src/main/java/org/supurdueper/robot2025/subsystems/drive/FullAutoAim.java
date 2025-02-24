package org.supurdueper.robot2025.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.subsystems.Vision;

public class FullAutoAim implements SwerveRequest {

    private RobotCentricFacingAngle robotCentricFacingAngle;

    private final PIDController m_pathXController = new PIDController(10, 0, 0);

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

    public FullAutoAim() {
        robotCentricFacingAngle = new RobotCentricFacingAngle();
        robotCentricFacingAngle.HeadingController.setPID(10, 0, .75);
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
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {

        // Set rotation based on what side of the reef we are facing. Also grab the april tag in view
        Translation2d reefCenter = AllianceFlip.apply(reefCenterBlue);
        Rotation2d facingReefCenter =
                reefCenter.minus(parameters.currentPose.getTranslation()).getAngle();
        robotCentricFacingAngle.TargetDirection =
                Collections.min(reefAngles, Comparator.comparing(angle -> absDistanceRadians(angle, facingReefCenter)));
        int aprilTagId = AllianceFlip.shouldFlip()
                ? reefAngleToAprilTagIdRed.get(robotCentricFacingAngle.TargetDirection)
                : reefAngleToAprilTagIdBlue.get(robotCentricFacingAngle.TargetDirection);

        // PID
        double error = Vision.getHorizonalOffsetFromTargetMeters(Vision.leftLimelightName);
        robotCentricFacingAngle.VelocityX = m_pathXController.calculate(error);

        return robotCentricFacingAngle.apply(parameters, modulesToApply);
    }

    private double absDistanceRadians(Rotation2d angle1, Rotation2d angle2) {
        return Math.abs(angle1.minus(angle2).getRadians());
    }
}
