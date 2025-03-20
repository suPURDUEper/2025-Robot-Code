package org.supurdueper.robtot2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.junit.jupiter.api.Test;
import org.supurdueper.robot2025.Constants.DriveConstants;
import org.supurdueper.robot2025.FieldConstants;

public class GetReefScoringPose {

    @Test
    void getReefScoringPoses() {
        Pose2d leftRobotScoringPose =
                new Pose2d(DriveConstants.robotToBumperCenter, DriveConstants.leftAutoAlignOffset, Rotation2d.k180deg);
        Pose2d rightRobotScoringPose =
                new Pose2d(DriveConstants.robotToBumperCenter, DriveConstants.rightAutoAlignOffset, Rotation2d.k180deg);
        int[] blueReefIds = FieldConstants.blueReefApriltagIds;
        for (int id : blueReefIds) {
            Pose2d leftScoringPose = FieldConstants.getAprilTagPose(id).plus(poseToTransform(leftRobotScoringPose));
            Pose2d rightScoringPose = FieldConstants.getAprilTagPose(id).plus(poseToTransform(rightRobotScoringPose));
            System.out.println(pose2dJson("Left_" + id, leftScoringPose));
            System.out.println(pose2dJson("Right_" + id, rightScoringPose));
        }
    }

    public static Transform2d poseToTransform(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    public static String pose2dJson(String name, Pose2d pose) {
        return """
                "%s":{
                    "x":{
                    "exp":"%.16f m",
                    "val":%.16f
                    },
                    "y":{
                    "exp":"%.16f m",
                    "val":%.16f
                    },
                    "heading":{
                    "exp":"%.16f rad",
                    "val":%.16f
                    }
                },
                """
                .formatted(
                        name,
                        pose.getX(),
                        pose.getX(),
                        pose.getY(),
                        pose.getY(),
                        pose.getRotation().getRadians(),
                        pose.getRotation().getRadians());
    }
}
