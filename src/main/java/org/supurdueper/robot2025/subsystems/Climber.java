// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2025.Constants.ClimberConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.state.RobotStates;

public class Climber extends PositionSubsystem implements SupurdueperSubsystem {

    PositionVoltage pidTuning = new PositionVoltage(0);

    public Climber() {
        configureMotors();
        Robot.add(this);
    }

    public void bindCommands() {
        RobotStates.actionClimbPrep.onTrue(climbPrep());
        RobotStates.actionClimb.onTrue(home());
    }

    public Command climbPrep() {
        return goToPosition(kClimb).withName("Climber.ClimbPrep");
    }

    public Command clearFunnel() {
        return goToPosition(kClearFunnel).withName("Climber.ClearFunnel");
    }

    public Command home() {
        return goToPosition(kHome).withName("Climber.Home");
    }

    public Command runFowards() {
        return setVoltage(() -> 12);
    }

    public Command runBackwards() {
        return setVoltage(() -> -12);
    }

    public Command stopCommand() {
        return run(this::stop).withName("Climber.Stop");
    }

    public void zero() {
        motor.setPosition(0);
    }

    @Override
    public Command goToPosition(Angle motorRotations) {
        return run(() -> motor.setControl(pidTuning.withPosition(motorRotations)));
    }

    @Override
    public Command goToPosition(double motorRotations) {
        return run(() -> motor.setControl(pidTuning.withPosition(motorRotations)));
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("Climber/Position", motor.getPosition().getValueAsDouble());
        DogLog.log("Climber/Setpoint", motor.getClosedLoopReference().getValueAsDouble());
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                // Idk what gravity type to use but this makes the most curent sense.
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
        return Rotations.of(kPositionTolerance);
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
        return CanId.TALONFX_CLIMBER_LEADER;
    }

    @Override
    public CanId canIdFollower() {
        return CanId.TALONFX_CLIMBER_FOLLOWER;
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
        return true;
    }
}
