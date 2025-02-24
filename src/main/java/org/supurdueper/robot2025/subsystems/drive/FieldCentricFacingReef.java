package org.supurdueper.robot2025.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.FieldConstants;

public class FieldCentricFacingReef extends FieldCentricFacingAngle {

    Translation2d reefCenterBlue = FieldConstants.Reef.center;
    final List<Rotation2d> reefAngles = List.of(
        Rotation2d.kZero,
        new Rotation2d(Math.PI / 3),
        new Rotation2d(2 * Math.PI / 3),
        Rotation2d.kPi,
        new Rotation2d(4 * Math.PI / 3),
        new Rotation2d(5 * Math.PI / 3)
    );

    public FieldCentricFacingReef() {
        HeadingController.setPID(10, 0, .75);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Translation2d reefCenter = AllianceFlip.apply(reefCenterBlue);
        Rotation2d facingReefCenter =
                reefCenter.minus(parameters.currentPose.getTranslation()).getAngle();
        TargetDirection =
                Collections.min(reefAngles, Comparator.comparing(angle -> absDistanceRadians(angle, facingReefCenter)));
        if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            // This is an angle from the frame of the reference of the field. Subtract
            // the operator persepctive to counteract CTRE adding it later
            TargetDirection = TargetDirection.minus(parameters.operatorForwardDirection);
        }
        return super.apply(parameters, modulesToApply);
    }

    private double absDistanceRadians(Rotation2d angle1, Rotation2d angle2) {
        return Math.abs(angle1.minus(angle2).getRadians());
    }
}
