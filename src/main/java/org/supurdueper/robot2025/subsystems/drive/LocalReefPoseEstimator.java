package org.supurdueper.robot2025.subsystems.drive;

import org.supurdueper.robot2025.subsystems.Vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LocalReefPoseEstimator extends SubsystemBase {

       // This is a local pose estimator that defines (0,0,0) to be the Apriltag in
    // question
    SwerveDrivePoseEstimator m_poseEstimator;

    private Drivetrain drivetrain;
    private Vision vision;
    private String limelightName;

    public LocalReefPoseEstimator(Drivetrain drivetrain, Vision vision, String limelightName) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.limelightName = limelightName;
        m_poseEstimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(),
                drivetrain.getPigeon2().getRotation2d(),
                drivetrain.getState().ModulePositions,
                vision.getRobotPoseTargetSpace(limelightName));

        drivetrain.registerTelemetry(state -> {
            m_poseEstimator.update(state.RawHeading, state.ModulePositions);
        });
    }

    
}
