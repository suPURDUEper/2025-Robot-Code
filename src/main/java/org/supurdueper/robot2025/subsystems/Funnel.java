// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static org.supurdueper.robot2025.Constants.FunnelConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.state.RobotStates;

public class Funnel extends TalonFXSubsystem implements SupurdueperSubsystem {

    /** Creates a new CoralIntake. */
    public Funnel() {
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void bindCommands() {
        RobotStates.actionIntake.onTrue(intake().until(RobotStates.hasCoral));
        RobotStates.actionUnjam.whileTrue(unjam());
        RobotStates.actionL1.onTrue(intake());
        RobotStates.actionScore.and(RobotStates.atL1).onTrue(stopCommand());
        RobotStates.actionScore.and(RobotStates.atL1).onFalse(unjam().withTimeout(1));
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    private void run() {
        runVoltage(kIntakeVoltage);
    }

    private void runReverse() {
        runVoltage(kUnjamVoltage);
    }

    public Command intake() {
        return runEnd(this::run, this::stop).withName("Funnel.Intake");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop);
    }
    // trying to make the intake motors wait until everything is at position

    public Command unjam() {
        return runEnd(this::runReverse, this::stop).withName("Funnel.Unjam");
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
        return kCurrentLimit;
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
