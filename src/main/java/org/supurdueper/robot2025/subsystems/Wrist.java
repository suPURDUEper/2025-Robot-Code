// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static org.supurdueper.robot2025.Constants.WristConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;

public class Wrist extends PositionSubsystem {

    private final CANcoder wristCancoder;

    public Wrist() {
        super();
        wristCancoder = new CANcoder(CanId.CANCODER_WRIST.getDeviceNumber(), CanId.CANCODER_WRIST.getBus());
        var fx_cfg = new TalonFXConfiguration();
        fx_cfg.Feedback.FeedbackRemoteSensorID = wristCancoder.getDeviceID();
        fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config = config.withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(CanId.CANCODER_WRIST.getDeviceNumber())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));
        configureMotors();
    }
//might be wrong about this
    private Angle wristRotationToDegrees(Angle wristAngle) {
        return Units.Degrees.of(wristAngle.in(Units.Degrees) * Constants.WristConstants.kDegreesPerRotation);
    }

    @Override
    public void periodic() {
        super.periodic();
        double wristPosition = wristRotationToDegrees(getPosition()).in(Units.Degrees);
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
        return null;
    }

    @Override
    public Angle positionTolerance() {
        return Constants.WristConstants.kPositionTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        return new SysIdRoutine(null, null);
    }

    @Override
    public CanId canIdLeader() {
        // TODO Auto-generated method stub
        return CanId.CANCODER_FUNNEL_TILT;
    }

    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.WristConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return false;
    }

    @Override
    public boolean brakeMode() {
        return true;
    }
}
