package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2025.Constants.DriveConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import org.supurdueper.lib.swerve.PhoenixProfiledPIDController;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.Constants.DriveConstants;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.RobotStates;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class BargeAutoAim implements SwerveRequest {

    private FieldCentricFacingAngle swerveRequest;
    private final PhoenixProfiledPIDController xController = new PhoenixProfiledPIDController(
            DriveConstants.translationKp,
            DriveConstants.translationKi,
            DriveConstants.translationKd,
            new TrapezoidProfile.Constraints(
                    TunerConstants.kMaxAutoAimSpeed.in(MetersPerSecond),
                    TunerConstants.kMaxAutoAimAcceleration.in(MetersPerSecondPerSecond)));

    private final double bargeXLocation = FieldConstants.startingLineX;
    private boolean reset = true;

    public BargeAutoAim() {
        swerveRequest = new FieldCentricFacingAngle();
        swerveRequest.HeadingController.setPID(
                DriveConstants.headingKp, DriveConstants.headingKi, DriveConstants.headingKd);
        swerveRequest.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
        swerveRequest.Deadband = translationClosedLoopDeadband.in(MetersPerSecond);
        swerveRequest.RotationalDeadband = rotationClosedLoopDeadband.in(RadiansPerSecond);
    }

    public void reset(SwerveControlParameters parameters, double goalX) {
        // Turn chassis speeds into field speeds
        if (reset) {
            ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                    parameters.currentChassisSpeed, parameters.currentPose.getRotation());

            xController.reset(parameters.currentPose.getX(), fieldVelocity.vxMetersPerSecond, parameters.timestamp);
            xController.setGoal(goalX);
        }
        reset = false;
    }

    public void setResetNextLoop() {
        reset = true;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Distance positionTolerance = Inches.of(1);
        // Set rotation towards oppposite all
        swerveRequest.TargetDirection = AllianceFlip.apply(Rotation2d.kZero);

        // Calculate goal pose
        double goalX = AllianceFlip.applyX(bargeXLocation);

        reset(parameters, goalX);

        // Calculate x and y velocities
        double distanceToGoalMeters = Math.abs(goalX - parameters.currentPose.getX());

        double driveVelocityMagnitude =
                xController.calculate(parameters.currentPose.getX(), goalX, parameters.timestamp);
        double ffMinRadius = 0.05;
        double ffMaxRadius = 1;
        double ffScaler = MathUtil.clamp((distanceToGoalMeters - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
        driveVelocityMagnitude += xController.getSetpoint().velocity * ffScaler;
        // Check if we're aimed
        if (distanceToGoalMeters < positionTolerance.in(Meters)) {
            RobotStates.setAimed(true);
            driveVelocityMagnitude = 0;
        } else {
            RobotStates.setAimed(false);
        }
        DogLog.log("Barge Auto Aim/Distance To Goal", distanceToGoalMeters);
        DogLog.log("Barge Auto Aim/Throttle Setpoint", xController.getSetpoint().position);

        swerveRequest.VelocityY =
                RobotContainer.getDriver().getDriveLeftPositive() * TunerConstants.kMaxSpeed.in(MetersPerSecond);
        if (AllianceFlip.shouldFlip()) {
            swerveRequest.VelocityY = swerveRequest.VelocityY * -1;
        }
        swerveRequest.VelocityX = driveVelocityMagnitude;

        // Log setpoint
        Pose2d lastSetpoint = new Pose2d(
                xController.getSetpoint().position + goalX,
                parameters.currentPose.getY(),
                swerveRequest.TargetDirection);
        DogLog.log("Auto Aim/Setpoint Position", lastSetpoint);

        return swerveRequest.withDriveRequestType(DriveRequestType.Velocity).apply(parameters, modulesToApply);
    }
}
