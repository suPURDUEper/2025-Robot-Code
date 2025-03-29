package org.supurdueper.robot2025.subsystems.drive;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.supurdueper.lib.swerve.DriveToPose;
import org.supurdueper.robot2025.Constants.DriveConstants;
import org.supurdueper.robot2025.FieldConstants;

public class AutoAim extends Command {

    private final Transform2d leftRobotScoringOffset =
            new Transform2d(DriveConstants.robotToBumperCenter, DriveConstants.leftAutoAlignOffset, Rotation2d.k180deg);
    private final Transform2d rightRobotScoringOffset = new Transform2d(
            DriveConstants.robotToBumperCenter, DriveConstants.rightAutoAlignOffset, Rotation2d.k180deg);
    private final Supplier<DriveToPose> driveToPose;
    private boolean isLeft;
    private Pose2d goalPose;
    private Drivetrain drivetrain;
    private boolean firstRun;

    public AutoAim(Drivetrain drivetrain, boolean isLeft, Supplier<DriveToPose> request) {
        this.isLeft = isLeft;
        this.driveToPose = request;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Calculate goal pose
        Rotation2d goalAngle =
                FieldConstants.getClosestReefAngle(drivetrain.getState().Pose.getTranslation());
        Transform2d offset = isLeft ? leftRobotScoringOffset : rightRobotScoringOffset;
        Pose2d aprilTagPose = FieldConstants.getAprilTagPose(FieldConstants.getReefTagId(goalAngle));
        Pose2d goalPose = new Pose2d(aprilTagPose.plus(offset).getTranslation(), goalAngle);
        DogLog.log("Auto Aim/Goal Position", goalPose);
        firstRun = true;
    }

    @Override
    public void execute() {
        DriveToPose drive = driveToPose.get();
        if (firstRun) {
            drive.resetNextLoop();
        }
        drivetrain.setControl(drive.withGoal(goalPose));
        firstRun = false;
    }
}
