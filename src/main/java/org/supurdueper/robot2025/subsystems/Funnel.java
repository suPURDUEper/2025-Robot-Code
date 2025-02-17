// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;
import org.supurdueper.robot2025.state.RobotStates;

public class Funnel extends TalonFXSubsystem implements SupurdueperSubsystem {

    /** Creates a new CoralIntake. */
    public Funnel() {
        configureMotors();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    private void run() {
        runVoltage(Constants.FunnelConstants.kIntakeVoltage);
    }

    private void runReverse() {
        runVoltage(Constants.FunnelConstants.kUnjamVoltage);
    }

    public Command intake() {
        return Commands.runEnd(this::run, this::stop, this);
    }

    public Command unjam() {
        return Commands.runEnd(this::runReverse, this::stop, this);
    }

    @Override
    public CanId canIdLeader() {
        return CanId.TALONFX_FUNNEL;
    }

    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.CoralScoreConstants.kCurrentLimit;
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
        RobotStates.actionIntake.onTrue(intake());
        RobotStates.actionUnjamIntake.onTrue(unjam());
    }

    @Override
    public boolean followerInverted() {
        return false;
    }
}
