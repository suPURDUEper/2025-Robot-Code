// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.supurdueper.robot2025.Constants.WristConstants.*;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.state.RobotStates;

public class Wrist extends PositionSubsystem implements SupurdueperSubsystem {

    private final CANcoder wristCancoder;
    private final PositionVoltage noMagicMotion = new PositionVoltage(0);

    public Wrist() {
        super();
        wristCancoder = new CANcoder(CanId.CANCODER_WRIST.getDeviceNumber(), CanId.CANCODER_WRIST.getBus());
        MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs()
                .withMagnetOffset(kCancoderMagnetOffset)
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        wristCancoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(cancoderConfig));

        config = config.withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(CanId.CANCODER_WRIST.getDeviceNumber())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));
        configureMotors();
        Robot.add(this);
    }

    public Command l1() {
        return goToPosition(kL1Angle).withName("Wrist.L1");
    }

    public Command l2() {
        return goToPosition(kL2Angle).withName("Wrist.L2");
    }

    public Command l3() {
        return goToPosition(kL3Angle).withName("Wrist.L3");
    }

    public Command l4() {
        return goToPosition(kL4Angle).withName("Wrist.L4");
    }

    public Command net() {
        return goToPosition(kNetAngle).withName("Wrist.Net");
    }

    public Command processor() {
        return goToPosition(kProcessorAngle).withName("Wrist.Processor");
    }

    public Command intake() {
        return goToPosition(kIntakeAngle).withName("Wrist.Intake");
    }

    public Command home() {
        return goToPosition(kHomeAngle).withName("Wrist.Home");
    }

    @Override
    public void bindCommands() {
        RobotStates.actionL1.onTrue(l1());
        RobotStates.actionL2.onTrue(l2());
        RobotStates.actionL3.onTrue(l3());
        RobotStates.actionL4.onTrue(l4());
        RobotStates.actionProcessor.onTrue(processor());
        RobotStates.actionNet.onTrue(Commands.waitSeconds(0.5).andThen(net()));
        RobotStates.actionIntake.onTrue(intake());
        RobotStates.actionHome.onTrue(home());
        RobotStates.actionScore.onFalse(home());
    }

    @Override
    public Command goToPosition(Angle rotations) {
        return run(() -> motor.setControl(noMagicMotion.withPosition(rotations)));
    }

    @Override
    public void periodic() {
        super.periodic();
        double wristPosition = getPosition().in(Units.Degrees);
        double wristTarget = getSetpoint().in(Units.Degrees);
        DogLog.log("Wrist/Position", wristPosition);
        DogLog.log("Wrist/Target", wristTarget);
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
        return new MotionMagicConfigs().withMotionMagicExpo_kA(profileKv).withMotionMagicExpo_kV(profileKa);
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
        return kPositionTolerance;
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
        return CanId.TALONFX_WRIST;
    }

    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return false;
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
