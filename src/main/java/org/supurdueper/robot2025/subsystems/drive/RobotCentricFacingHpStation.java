package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2025.Constants.DriveConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;
import edu.wpi.first.math.geometry.Rotation2d;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.FieldConstants;

public class RobotCentricFacingHpStation extends RobotCentricFacingAngle {

    final Rotation2d leftStation = FieldConstants.CoralStation.leftCenterFace.getRotation();
    final Rotation2d rightStation = FieldConstants.CoralStation.rightCenterFace.getRotation();

    public RobotCentricFacingHpStation() {
        HeadingController.setPID(headingKp, headingKi, headingKd);
        RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        boolean scoringTableSide = parameters.currentPose.getTranslation().getY() <= FieldConstants.fieldWidth / 2;
        if (scoringTableSide) {
            if (AllianceFlip.shouldFlip()) {
                TargetDirection = leftStation;
            } else {
                TargetDirection = rightStation;
            }
        } else {
            if (AllianceFlip.shouldFlip()) {
                TargetDirection = rightStation;
            } else {
                TargetDirection = leftStation;
            }
        }
        return super.apply(parameters, modulesToApply);
    }
}
