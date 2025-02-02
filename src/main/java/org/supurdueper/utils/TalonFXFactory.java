package org.supurdueper.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import org.supurdueper.robot2025.CanId;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory {

    private static NeutralModeValue neutralMode = NeutralModeValue.Brake;
    private static InvertedValue invertValue = InvertedValue.CounterClockwise_Positive;
    private static double neutralDeadband = 0.04;
    private static double supplyCurrentLimit = 40;

    private TalonFXFactory() {}

    // create a CANTalon with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(CanId id) {
        var talon = createTalon(id);
        talon.getConfigurator().apply(getDefaultConfig());
        return talon;
    }

    public static TalonFX createConfigTalon(CanId id, TalonFXConfiguration config) {
        var talon = createTalon(id);
        talon.getConfigurator().apply(config);
        return talon;
    }

    // Create a new follower talon with same configuration as the leader talon
    public static TalonFX createPermanentFollowerTalon(
            CanId followerId, TalonFX leaderTalonFX, boolean opposeLeaderDirection) {
        String leaderCanBus = leaderTalonFX.getNetwork();
        int leaderId = leaderTalonFX.getDeviceID();
        if (!followerId.getBus().equals(leaderCanBus)) {
            throw new RuntimeException("Leader and Follwer Talons must be on the same CAN bus");
        }

        TalonFXConfiguration followerConfig = getDefaultConfig();
        leaderTalonFX.getConfigurator().refresh(followerConfig);
        final TalonFX talon = createConfigTalon(followerId, followerConfig);

        talon.setControl(new Follower(leaderId, opposeLeaderDirection));
        return talon;
    }

    public static TalonFXConfiguration getDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = invertValue;
        config.MotorOutput.DutyCycleNeutralDeadband = neutralDeadband;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;

        config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.FeedbackRotorOffset = 0;
        config.Feedback.SensorToMechanismRatio = 1;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        config.Audio.BeepOnBoot = true;
        config.Audio.AllowMusicDurDisable = true;
        config.Audio.BeepOnConfig = true;

        return config;
    }

    private static TalonFX createTalon(CanId id) {
        TalonFX talon = new TalonFX(id.getDeviceNumber(), id.getBus());
        talon.clearStickyFaults();

        return talon;
    }
}
