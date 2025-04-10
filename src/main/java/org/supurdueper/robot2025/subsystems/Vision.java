// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.supurdueper.lib.LimelightHelpers;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.subsystems.drive.Drivetrain;

public class Vision extends SubsystemBase implements SupurdueperSubsystem {

    public static final String leftLimelightName = "limelight-fl";
    public static final String rightLimelimeName = "limelight-fr";
    public static final String backLimelightName = "limelight-r";

    public Vision() {
        Robot.add(this);
    }

    @Override
    public void periodic() {
        Drivetrain drivetrain = RobotContainer.getDrivetrain();
        SwerveDriveState state = drivetrain.getState();
        updatePose3dAprilTag(leftLimelightName, drivetrain, state);
        updatePose3dAprilTag(rightLimelimeName, drivetrain, state);
        updatePose3dAprilTag(backLimelightName, drivetrain, state);
    }

    private void updatePose3dAprilTag(String limelightName, Drivetrain drivetrain, SwerveDriveState state) {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        boolean shouldUpdate = true;
        if (Math.abs(state.Speeds.omegaRadiansPerSecond) > Units.degreesToRadians(360)) {
            shouldUpdate = false;
        }
        if (mt2 == null) {
            shouldUpdate = false;
            return;
        }
        if (mt2.tagCount == 0) {
            shouldUpdate = false;
        }
        if (mt2.rawFiducials.length == 1) {
            double ambiguity = mt2.rawFiducials[0].ambiguity;
            if (ambiguity >= .7) {
                shouldUpdate = false;
            }
        }
        if (Math.abs(mt2.pose.getRotation().minus(state.Pose.getRotation()).getDegrees()) > 30) { // 1 meter
            shouldUpdate = false;
        }
        if (shouldUpdate) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
            drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
        DogLog.log("Vision/" + limelightName + " Pose (mt2)", mt2.pose);
    }

    public static void updateIMUMode() {
        LimelightHelpers.SetIMUMode(leftLimelightName, 3);
    }

    public static void setDisabled() {
        LimelightHelpers.SetThrottle(leftLimelightName, 150);
    }

    public static void setEnabled() {
        LimelightHelpers.SetThrottle(leftLimelightName, 0);
    }

    public static void setAprilTagFilter() {
        int[] ids = AllianceFlip.shouldFlip() ? FieldConstants.redReefApriltagIds : FieldConstants.blueReefApriltagIds;
        LimelightHelpers.SetFiducialIDFiltersOverride(leftLimelightName, ids);
        LimelightHelpers.SetFiducialIDFiltersOverride(rightLimelimeName, ids);
        LimelightHelpers.SetFiducialIDFiltersOverride(backLimelightName, ids);
    }

    @Override
    public void bindCommands() {}
}
