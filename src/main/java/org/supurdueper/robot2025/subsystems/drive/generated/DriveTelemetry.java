package org.supurdueper.robot2025.subsystems.drive.generated;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.supurdueper.robot2025.subsystems.Vision;

public class DriveTelemetry {
    // kSpeedAt12Volts desired top speed
    private final double MaxSpeed = TunerConstants.kMaxSpeed.in(MetersPerSecond);

    /** Construct a telemetry object, with the specified max speed of the robot */
    public DriveTelemetry() {
        SignalLogger.start();
    }

    // /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // /* Robot swerve drive state */
    // private final NetworkTable driveStateTable = inst.getTable("DriveState");
    // private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct)
    //         .publish();
    // private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable
    //         .getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    // private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
    //         .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
    //         .publish();
    // private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable
    //         .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
    //         .publish();
    // private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
    //         .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
    //         .publish();
    // private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    // private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency")
    //         .publish();

    // /* Robot pose for field positioning */
    // private final NetworkTable table = inst.getTable("Pose");
    // private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    // private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    // /* Mechanisms to represent the swerve module states */
    // private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
    //         new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
    // };
    // /* A direction and length changing ligament for speed representation */
    // private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
    //         m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5)
    //                 .append(new MechanismLigament2d("Speed", 0.5, 0)),
    //         m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5)
    //                 .append(new MechanismLigament2d("Speed", 0.5, 0)),
    //         m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5)
    //                 .append(new MechanismLigament2d("Speed", 0.5, 0)),
    //         m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5)
    //                 .append(new MechanismLigament2d("Speed", 0.5, 0)),
    // };
    // /* A direction changing and length constant ligament for module direction */
    // private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
    //         m_moduleMechanisms[0]
    //                 .getRoot("RootDirection", 0.5, 0.5)
    //                 .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
    //                         new Color8Bit(Color.kWhite))),
    //         m_moduleMechanisms[1]
    //                 .getRoot("RootDirection", 0.5, 0.5)
    //                 .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
    //                         new Color8Bit(Color.kWhite))),
    //         m_moduleMechanisms[2]
    //                 .getRoot("RootDirection", 0.5, 0.5)
    //                 .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
    //                         new Color8Bit(Color.kWhite))),
    //         m_moduleMechanisms[3]
    //                 .getRoot("RootDirection", 0.5, 0.5)
    //                 .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
    //                         new Color8Bit(Color.kWhite))),
    // };

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    private final NetworkTable leftLimelightTable = inst.getTable(Vision.leftLimelightName);
    private final NetworkTable rightLimelightTable = inst.getTable(Vision.rightLimelimeName);
    private final DoubleArrayPublisher leftLimelightRobotOrientationPublisher =
            leftLimelightTable.getDoubleArrayTopic("robot_orientation_set").publish();
    private final DoubleArrayPublisher rightLimelightRobotOrientationPublisher =
            rightLimelightTable.getDoubleArrayTopic("robot_orientation_set").publish();
    private double[] robotOrientation = {0, 0, 0, 0, 0, 0};
    private double[] wrappedModuleAngleRads = {0, 0, 0, 0};

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        // drivePose.set(state.Pose);
        // driveSpeeds.set(state.Speeds);
        // driveModuleStates.set(state.ModuleStates);
        // driveModuleTargets.set(state.ModuleTargets);
        // driveModulePositions.set(state.ModulePositions);
        // driveTimestamp.set(state.Timestamp);
        // driveOdometryFrequency.set(1.0 / state.OdometryPeriod);
        robotOrientation[0] = state.Pose.getRotation().getDegrees();
        leftLimelightRobotOrientationPublisher.set(robotOrientation);
        rightLimelightRobotOrientationPublisher.set(robotOrientation);

        // /* Also write to log file */
        // m_poseArray[0] = state.Pose.getX();
        // m_poseArray[1] = state.Pose.getY();
        // m_poseArray[2] = state.Pose.getRotation().getDegrees();
        // for (int i = 0; i < 4; ++i) {
        //     m_moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
        //     m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
        //     m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
        //     m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        // }

        DogLog.log("Drive/Pose", state.Pose);
        DogLog.log("Drive/ModuleStates", state.ModuleStates);
        DogLog.log("Drive/ModuleTargets", state.ModuleTargets);
        DogLog.log("Drive/MeasuredSpeeds", state.Speeds);
        DogLog.log("Drive/OdometryPeriod", state.OdometryPeriod);
        for (int i = 0; i < 4; i++) {
            wrappedModuleAngleRads[i] = MathUtil.angleModulus(state.ModuleStates[i].angle.getRadians());
        }
        DogLog.log("Drive/ModuleAnglesWrapped", wrappedModuleAngleRads);

        // /* Telemeterize the pose to a Field2d */
        // fieldTypePub.set("Field2d");
        // fieldPub.set(m_poseArray);

        // /* Telemeterize the module states to a Mechanism2d */
        // if (Constants.publishToNT) {
        //     for (int i = 0; i < 4; ++i) {
        //         m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
        //         m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
        //         m_moduleSpeeds[i]
        //                 .setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

        //         SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        //     }
        // }
    }
}
