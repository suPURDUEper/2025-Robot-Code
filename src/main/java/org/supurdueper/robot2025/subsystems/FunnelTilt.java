package org.supurdueper.robot2025.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.supurdueper.robot2025.Constants.FunnelTiltConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;
import org.supurdueper.robot2025.Robot;

public class FunnelTilt extends PositionSubsystem implements SupurdueperSubsystem {

    private final PositionVoltage noMagicMotion = new PositionVoltage(0);
    private DutyCycleEncoder absEncoder;

    public FunnelTilt() {
        absEncoder = new DutyCycleEncoder(1);
        config = config.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(kSensorToMechanismRatio));
        configureMotors();
        Robot.add(this);
    }

    public Angle getAbsEncoder() {
        return Rotations.of(absEncoder.get() * (48.0 / 80.0)).minus(kAbsEncoderOffset);
    }

    public Command goToIntakeAngle() {
        return goToPosition(kIntakePosition);
    }

    public Command goToStartingPosition() {
        return goToPosition(kStartPosition);
    }

    @Override
    public Command goToPosition(Angle rotations) {
        return Commands.run(() -> motor.setControl(noMagicMotion.withPosition(rotations)));
    }

    @Override
    public void periodic() {
        super.periodic();
        double wristPosition = getPosition().in(Units.Degrees);
        double wristTarget = getSetpoint().in(Units.Degrees);

        SmartDashboard.putNumber("FunnelTilt/Position", wristPosition);
        SmartDashboard.putNumber("FunnelTilt/AbsEncoder", getAbsEncoder().in(Degrees));
        SmartDashboard.putNumber("FunnelTilt/Target", wristTarget);
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
    public void bindCommands() {
        motor.setPosition(getAbsEncoder());
    }

    @Override
    public boolean followerInverted() {
        return false;
    }
}
