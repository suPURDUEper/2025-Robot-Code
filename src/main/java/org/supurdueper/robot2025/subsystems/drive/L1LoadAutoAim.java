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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import org.supurdueper.lib.swerve.PhoenixProfiledPIDController;
import org.supurdueper.lib.utils.AllianceFlip;
import org.supurdueper.robot2025.Constants.DriveConstants;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class L1LoadAutoAim implements SwerveRequest {

    private RobotCentricFacingHpStation swerveRequest;
    private final PhoenixProfiledPIDController throttleController = new PhoenixProfiledPIDController(
            DriveConstants.translationKp,
            DriveConstants.translationKi,
            DriveConstants.translationKd,
            new TrapezoidProfile.Constraints(
                    TunerConstants.kMaxAutoAimSpeed.in(MetersPerSecond),
                    TunerConstants.kMaxAutoAimAcceleration.in(MetersPerSecondPerSecond)));

    private boolean reset = true;

    public L1LoadAutoAim() {
        swerveRequest = new RobotCentricFacingHpStation();
        swerveRequest.ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;
        swerveRequest.Deadband = translationClosedLoopDeadband.in(MetersPerSecond);
    }

    public void reset(SwerveControlParameters parameters, Distance goal) {
        // Turn chassis speeds into field speeds
        if (reset) {
            throttleController.reset(
                    RobotContainer.getElevator().distanceFromReef().in(Meters),
                    parameters.currentChassisSpeed.vxMetersPerSecond,
                    parameters.timestamp);
            throttleController.setGoal(goal.in(Meters));
        }
        reset = false;
    }

    public void setResetNextLoop() {
        reset = true;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Distance positionTolerance = Inches.of(1);

        // Calculate goal pose
        Distance goal = Inches.of(10);
        Distance current = RobotContainer.getElevator().distanceFromReef();
        double distanceToGoalMeters = Math.abs(goal.minus(current).in(Meters));
        reset(parameters, goal);

        // Calculate x and y velocities
        double driveVelocityMagnitude =
                throttleController.calculate(current.in(Meters), goal.in(Meters), parameters.timestamp);
        double ffMinRadius = 0.05;
        double ffMaxRadius = 1;
        double ffScaler = MathUtil.clamp((distanceToGoalMeters - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
        driveVelocityMagnitude += throttleController.getSetpoint().velocity * ffScaler;
        // Check if we're aimed
        if (distanceToGoalMeters < positionTolerance.in(Meters)) {
            driveVelocityMagnitude = 0;
        }
        DogLog.log("L1 Load Aim/Distance To Goal", distanceToGoalMeters);
        DogLog.log("L1 Load Aim/Throttle Setpoint", throttleController.getSetpoint().position);

        swerveRequest.VelocityX = driveVelocityMagnitude;
        swerveRequest.VelocityY =
                RobotContainer.getDriver().getDriveLeftPositive() * TunerConstants.kMaxSpeed.in(MetersPerSecond);
        if (AllianceFlip.shouldFlip()) {
            swerveRequest.VelocityY = swerveRequest.VelocityY * -1;
        }

        return swerveRequest.withDriveRequestType(DriveRequestType.Velocity).apply(parameters, modulesToApply);
    }
}
