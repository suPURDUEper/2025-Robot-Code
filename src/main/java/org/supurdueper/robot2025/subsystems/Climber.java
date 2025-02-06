// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;

public class Climber extends PositionSubsystem {

    public Climber() {
        configureMotors();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs();
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs();
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return new SoftwareLimitSwitchConfigs();
    }

    @Override
    public Angle positionTolerance() {
        return Constants.ClimberConstants.kPositionTolerance;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        return new SysIdRoutine(null, null);
    }

    @Override
    public CanId canIdLeader() {
        return CanId.TALONFX_CLIMBER_LEADER;
    }

    @Override
    public CanId canIdFollower() {
        return CanId.TALONFX_CLIMBER_FOLLOWER;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.ClimberConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return true;
    }

    @Override
    public boolean brakeMode() {
        return true;
    }
}
