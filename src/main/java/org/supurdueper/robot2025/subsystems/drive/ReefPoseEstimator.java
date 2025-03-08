package org.supurdueper.robot2025.subsystems.drive;

import java.util.Optional;

import org.supurdueper.robot2025.subsystems.Vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Setter;

public class ReefPoseEstimator {

    // This is a local pose estimator that defines (0,0,0) to be the Apriltag in
    // question
    private SwerveDrivePoseEstimator m_poseEstimator;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(2.0);
    private Drivetrain drivetrain;
    private Vision vision;
    private String limelightName;

    @Setter
    private boolean active;

    public ReefPoseEstimator(Drivetrain drivetrain, Vision vision, String limelightName) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.limelightName = limelightName;
        m_poseEstimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(),
                drivetrain.getPigeon2().getRotation2d(),
                drivetrain.getState().ModulePositions,
                Vision.getRobotPoseTargetSpace(limelightName));
    }

    public void update(SwerveDriveState state) {
        poseBuffer.addSample(state.Timestamp, state.Pose);
        if (active) {
            m_poseEstimator.updateWithTime(state.Timestamp, state.RawHeading, state.ModulePositions);
        }
    }

    public void updateVision() {
        m_poseEstimator.addVisionMeasurement(Vision.getRobotPoseTargetSpace(limelightName), 0);
    }

    public void reset(SwerveDriveState state) {
        m_poseEstimator.resetPosition(state.RawHeading, state.ModulePositions, Vision.getRobotPoseTargetSpace(limelightName));
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

//     public void addTxTyObservation(TxTyObservation observation) {

//     // Get odometry based pose at timestamp
//     var sample = poseBuffer.getSample(observation.timestamp());
//     if (sample.isEmpty()) {
//       // exit if not there
//       return;
//     }

//     // Average tx's and ty's
//     double tx = 0.0;
//     double ty = 0.0;
//     for (int i = 0; i < 4; i++) {
//       tx += observation.tx()[i];
//       ty += observation.ty()[i];
//     }
//     tx /= 4.0;
//     ty /= 4.0;

//     Pose3d cameraPose = Vision.cameraPoses[observation.camera()];
//     // Use 2d distance and tag angle + tx to find robot pose
//     double distance2d = observation.distance() * Math.cos(-cameraPose.getRotation().getY() - ty);
//     Rotation2d camToTagRotation =
//         sample
//             .get()
//             .getRotation()
//             .plus(cameraPose.toPose2d().getRotation().plus(Rotation2d.fromRadians(-tx)));
//     var tagPose2d = tagPoses2d.get(observation.tagId());
//     if (tagPose2d == null) return;
//     Translation2d fieldToCameraTranslation =
//         new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
//             .transformBy(GeomUtil.toTransform2d(distance2d, 0.0))
//             .getTranslation();
//     Pose2d robotPose =
//         new Pose2d(
//                 fieldToCameraTranslation,
//                 sample.get().getRotation().plus(cameraPose.toPose2d().getRotation()))
//             .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
//     // Use gyro angle at time for robot rotation
//     robotPose = new Pose2d(robotPose.getTranslation(), sample.get().getRotation());

//     // Add transform to current odometry based pose for latency correction
//     txTyPoses.put(
//         observation.tagId(), new TxTyPoseRecord(robotPose, distance2d, observation.timestamp()));
//   }

//   /** Get 2d pose estimate of robot if not stale. */
//   public Optional<Pose2d> getTxTyPose(int tagId) {
//     if (!txTyPoses.containsKey(tagId)) {
//       DriverStation.reportError("No tag with id: " + tagId, true);
//       return Optional.empty();
//     }
//     var data = txTyPoses.get(tagId);
//     // Check if stale
//     if (Timer.getTimestamp() - data.timestamp() >= txTyObservationStaleSecs.get()) {
//       return Optional.empty();
//     }
//     // Get odometry based pose at timestamp
//     var sample = poseBuffer.getSample(data.timestamp());
//     // Latency compensate
//     return sample.map(pose2d -> data.pose().plus(new Transform2d(pose2d, odometryPose)));
//   }

}