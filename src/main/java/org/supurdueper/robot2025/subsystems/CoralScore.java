// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;
import org.supurdueper.robot2025.state.RobotStates;

public class CoralScore extends TalonFXSubsystem implements SupurdueperSubsystem {

    private DigitalInput coralScoreBB;
    public boolean hasCoral;

    public CoralScore() {
        coralScoreBB = new DigitalInput(Constants.DIOport.scoreBreakbeam1);
        configureMotors();
    }

    public Trigger coralLoaded() {
        return new Trigger(this::hasCoral);
    }

    public boolean hasCoral() {
        return !coralScoreBB.get();
    }

    private boolean scoredCoral() {
        return coralScoreBB.get();
    }

    public Command loadCoral() {
        return Commands.runEnd(this::load, this::stop).until(this::hasCoral);
    }

    private void load() {
        runVoltage(Constants.CoralScoreConstants.loadVoltage);
    }

    public Command l4( ) {
        return Commands.runEnd(this::scoreL4, this::stop).until(this::scoredCoral);
    }

    private void scoreL4() {
        runVoltage(Constants.CoralScoreConstants.scoreL4Voltage);
    } 

    public Command l2L3( ) {
        return Commands.runEnd(this::scoreL1, this::stop).until(this::scoredCoral);
    }

    private void scoreL2L3() {
        runVoltage(Constants.CoralScoreConstants.scoreL2L3Voltage);
    } 

    public Command l1( ) {
        return Commands.runEnd(this::scoreL1, this::stop).until(this::scoredCoral);
    }

    private void scoreL1() {
        runVoltage(Constants.CoralScoreConstants.scoreL1Voltage);
    } 

    public Command unJam() {
        return Commands.runEnd(this::runBackwards, this::stop);
    }

    private void runBackwards() {
        runVoltage(Constants.CoralScoreConstants.unJamVoltage);
    }

    @Override
    public void bindCommands() {
        RobotStates.actionIntake.onTrue(loadCoral());
        RobotStates.actionScore.and(RobotStates.atL1).onTrue(l1());
        RobotStates.actionScore.and(RobotStates.atL2.or(RobotStates.atL3)).onTrue(l2L3());
        RobotStates.actionScore.and(RobotStates.atL4).onTrue(l4());
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("CoralScore/BreakBeam1", coralScoreBB.get());
    }

    @Override
    public CanId canIdLeader() {
        return CanId.TALONFX_CORAL;
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
        return true;
    }

    @Override
    public boolean followerInverted() {
        return false;
    }
}
