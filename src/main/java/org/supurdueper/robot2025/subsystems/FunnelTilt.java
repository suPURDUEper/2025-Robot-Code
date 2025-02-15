package org.supurdueper.robot2025.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.supurdueper.robot2025.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;

public class FunnelTilt extends PositionSubsystem implements SupurdueperSubsystem {

    public FunnelTilt() {
        configureMotors();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
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
        return CanId.CANCODER_FUNNEL_TILT;
    }

    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        // TODO Auto-generated method stub
        return Constants.CoralScoreConstants.kCurrentLimit;
    }

    @Override
    public boolean inverted() {
        return true;
    }

    @Override
    public boolean brakeMode() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void bindCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'bindCommands'");
    }

    @Override
    public boolean followerInverted() {
        return false;
    }
}
