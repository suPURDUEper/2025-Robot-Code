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

public class FunnelTilt extends PositionSubsystem {

    public FunnelTilt() {
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'canIdLeader'");
    }

    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'currentLimits'");
    }

    @Override
    public boolean inverted() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'inverted'");
    }

    @Override
    public boolean brakeMode() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'brakeMode'");
    }
}
