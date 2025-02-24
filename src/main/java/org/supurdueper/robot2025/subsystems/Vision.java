// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;

import org.supurdueper.lib.LimelightHelpers;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.subsystems.drive.Drivetrain;

public class Vision extends SubsystemBase {

    public static final String leftLimelightName = "limelight-fl";
    public static final String rightLimelimeName = "limelight-fr";

    @Override
    public void periodic() {
        updatePose3dAprilTag(leftLimelightName);
        updatePose3dAprilTag(rightLimelimeName);
    }

    private void updatePose3dAprilTag(String limelightName) {
        Drivetrain drivetrain = RobotContainer.getDrivetrain();
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation(
                limelightName, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > Units.degreesToRadians(720)) {
            return;
        }
        if (mt2 == null) {
            return;
        }
        if (mt2.tagCount == 0) {
            return;
        }
        if (!doRejectUpdate) {
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
            drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
        DogLog.log("Vision/" + limelightName + " Pose (mt2)", mt2.pose);
    }

    public static double getTargetId(String limelightName) {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    public static double getHorizonalOffsetFromTargetMeters(String limelightName) {
        return LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getMeasureX().in(Meters);
    }
}
