package org.supurdueper.lib.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The SwerveRequest::apply function runs in a fast (250hz on CAN FD) thread that is timed on the CTRE StatusSignal API.
 * This is the same thread that updates odometry. By extending this request to set the target angle to face calculated
 * against a specified point on the field instead of using SwerveRequest.FieldCentricFacingAngle, we can ensure that
 * we're updating the target rotation at 250hz and always utilizing the latest pose estimation.
 */
public class FieldCentricFacingPoint extends FieldCentricFacingAngle {

    Translation2d pointToFace;

    public FieldCentricFacingPoint() {
        this.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        this.TargetDirection =
                pointToFace.minus(parameters.currentPose.getTranslation()).getAngle();
        return super.apply(parameters, modulesToApply);
    }

    public FieldCentricFacingAngle withPointToFace(Translation2d point) {
        this.pointToFace = point;
        return this;
    }

    public Rotation2d getTargetDirection() {
        return this.TargetDirection;
    }
}
