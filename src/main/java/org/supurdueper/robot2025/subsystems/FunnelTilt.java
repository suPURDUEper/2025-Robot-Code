package org.supurdueper.robot2025.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.supurdueper.robot2025.Constants.FunnelTiltConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.state.RobotStates;

public class FunnelTilt extends PositionSubsystem implements SupurdueperSubsystem {

    private final PositionVoltage noMagicMotion = new PositionVoltage(0);
    private final CANcoder funnelCancoder;

    public FunnelTilt() {
        funnelCancoder =
                new CANcoder(CanId.CANCODER_FUNNEL_TILT.getDeviceNumber(), CanId.CANCODER_FUNNEL_TILT.getBus());
        MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs()
                .withMagnetOffset(kAbsEncoderOffset)
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        funnelCancoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(cancoderConfig));

        config = config.withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(CanId.CANCODER_FUNNEL_TILT.getDeviceNumber())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withSensorToMechanismRatio(kAbsEncoderRatio));
        configureMotors();
        Robot.add(this);
    }

    public Command intake() {
        return goToPosition(() -> kIntakePosition).withName("FunnelTilt.Intake");
    }

    public Command l1() {
        return goToPosition(() -> kL1Position).withName("FunnelTilt.l1");
    }

    public Command startingPosition() {
        return goToPosition(() -> kStartPosition).withName("FunnelTilt.StartingPosition");
    }

    public Command climbPosition() {
        return goToPosition(() -> kClimbPosition).withName("FunnelTilt.ClimbPosition");
    }

    @Override
    public void bindCommands() {
        RobotStates.actionClimbPrep.onTrue(climbPosition());
        RobotStates.actionIntake.onTrue(intake());
        RobotStates.actionL1.onTrue(intake());
        RobotStates.actionScore.and(RobotStates.atL1).onTrue(l1());
    }

    @Override
    public Command goToPosition(Supplier<Angle> rotations) {
        return run(() -> motor.setControl(noMagicMotion.withPosition(rotations.get())));
    }

    @Override
    public void periodic() {
        super.periodic();

        double wristPosition = getPosition().in(Units.Degrees);
        double wristTarget = getSetpoint().in(Units.Degrees);
        DogLog.log("FunnelTilt/Position", wristPosition);
        DogLog.log("FunnelTilt/Target", wristTarget);
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(kp)
                .withKI(ki)
                .withKD(kd)
                .withKS(ks)
                .withKV(kv)
                .withKA(ka)
                .withKG(kg);
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs().withMotionMagicExpo_kV(profileKv).withMotionMagicExpo_kA(profileKa);
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(kForwardSoftLimit)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(kReverseSoftLimit)
                .withReverseSoftLimitEnable(true);
    }

    @Override
    public Angle positionTolerance() {
        return Constants.FunnelTiltConstants.kPositionTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        return new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        Volts.of(1).per(Second),
                        Volts.of(7),
                        null,
                        state -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motor(s).
                        this::runVoltage,
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being
                        // characterized.
                        null, // Using the CTRE SignalLogger API instead
                        // Tell SysId to make generated commands require this subsystem, suffix test
                        // state in
                        // WPILog with this subsystem's name ("shooter")
                        this));
    }

    @Override
    public CanId canIdLeader() {
        return CanId.TALONFX_FUNNEL_TILT;
    }

    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.FunnelTiltConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return true;
    }

    @Override
    public boolean brakeMode() {
        return true;
    }

    @Override
    public boolean followerInverted() {
        return false;
    }
}
