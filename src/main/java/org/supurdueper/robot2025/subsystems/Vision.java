// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.supurdueper.lib.LimelightHelpers;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.subsystems.drive.Drivetrain;

public class Vision extends SubsystemBase {

    //   private static final double kBufferDuration = 1.5;
    //   private TimeInterpolatableBuffer<Rotation2d> robotAngleBuffer;
    //   private int[] validIDs = {3, 4, 7, 8};
    //   private double[] visionPose = new double[3];

    /** Creates a new LimeLight. */
    public Vision() {
        // robotAngleBuffer = TimeInterpolatableBuffer.createBuffer(kBufferDuration);
    }

    @Override
    public void periodic() {
        // Update gyro angle buffer
        // robotAngleBuffer.addSample(
        //     MathSharedStore.getTimestamp(), drivetrain.getState().Pose.getRotation());
        // getDistanceToGoalMeters();
        // updatePose2DAprilTag();
        updatePose3dAprilTag("limelight-fl");
        updatePose3dAprilTag("limelight-fr");
    }

    private void updatePose3dAprilTag(String limelightName) {
        Drivetrain drivetrain = RobotContainer.getDrivetrain();
        boolean doRejectUpdate = false;
        // LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
        LimelightHelpers.SetRobotOrientation(
                limelightName, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        // if our angular velocity is greater than 720 degrees per second, ignore vision updates
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

    //   private void updatePose2DAprilTag() {
    //     // To do any processing we need to know what alliance we are on and if we have a valid target
    //     if (DriverStation.getAlliance().isEmpty() || !LimelightHelpers.getTV("")) {
    //       return;
    //     }
    //     double ty = LimelightHelpers.getTY("");
    //     double tx = LimelightHelpers.getTX("");
    //     double latencyMs =
    //         LimelightHelpers.getLatency_Capture("") + LimelightHelpers.getLatency_Pipeline("");

    //     if (latencyMs / 1000.0 > kBufferDuration) {
    //       return;
    //     }

    //     double timestamp = MathSharedStore.getTimestamp() - latencyMs / 1000.0;
    //     // Calculate how far we are from the apriltag
    //     double distance = getDistanceToGoalMeters().get();

    //     // Calculate the angle from the apriltag to the camera. Account for the delay in the camera
    //     // capture and processing
    //     Optional<Rotation2d> robotAngleOptional = robotAngleBuffer.getSample(timestamp);
    //     Rotation2d robotAngle =
    //         robotAngleOptional.orElseGet(() -> drivetrain.getState().Pose.getRotation());
    //     Rotation2d angle = robotAngle.minus(Rotation2d.fromDegrees(tx));
    //     SmartDashboard.putNumber("Vision/AngleToGoal (deg)", angle.getDegrees());

    //     // Make a vector using the previous distance and angle and calculate the camera position
    //     Translation2d apriltag =
    //         (DriverStation.getAlliance().get() == Alliance.Red)
    //             ? FieldConstants.aprilTag4
    //             : FieldConstants.aprilTag7;
    //     Translation2d tagToCameraTranslation = new Translation2d(distance, angle);
    //     Translation2d cameraToRobotCenterTranslation = new Translation2d(0, robotAngle);
    //     Translation2d robot =
    //         apriltag.plus(tagToCameraTranslation).plus(cameraToRobotCenterTranslation);
    //     Pose2d poseEstimate = new Pose2d(robot, robotAngle);
    //     SmartDashboard.putNumberArray(
    //         "Vision/Pose Estimate",
    //         new double[] {
    //           poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getRotation().getRotations()
    //         });
    //     drivetrain.addVisionMeasurement(poseEstimate, timestamp);
    //   }
}
