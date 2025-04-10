// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static org.supurdueper.robot2025.Constants.AlgaeScoreConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.state.RobotStates;

public class AlgaeScore extends TalonFXSubsystem implements SupurdueperSubsystem {

    public AlgaeScore() {
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void bindCommands() {
        setDefaultCommand(hold());
        RobotStates.atL2.or(RobotStates.atL3).whileTrue(intake());
        RobotStates.atL2.or(RobotStates.atL3).onFalse(hold());
        RobotStates.actionScore.and(RobotStates.atNet).onTrue(scoreNet());
        RobotStates.actionScore.and(RobotStates.atProcessor).onTrue(scoreProcessor());
        RobotStates.actionScore
                .and(RobotStates.atIntake.or(RobotStates.atL1).and(RobotStates.hasCoral.negate()))
                .onTrue(scoreProcessor());
        RobotStates.actionLollipop.onTrue(intake());
    }

    @Override
    public void periodic() {
        super.periodic();
        if (getCurrentCommand() != null && getCurrentCommand().getName() != null) {
            DogLog.log("AlgaeScore/Command", getCurrentCommand().getName());
        }
    }

    // Public methods
    public Command intake() {
        return run(this::runIntake).withName("AlgaeScore.Intake");
    }

    public Command hold() {
        return run(this::runHold).withName("AlgaeScore.Hold");
    }

    public Command scoreNet() {
        return runEnd(this::net, this::stop).withTimeout(kNetScoreTime).withName("AlgaeScore.ScoreNet");
    }

    public Command scoreProcessor() {
        return runEnd(this::processor, this::stop)
                .withTimeout(kProcessorScoreTime)
                .withName("AlgaeScore.ScoreProcessor");
    }

    private void runIntake() {
        runVoltage(kIntakeVoltage);
    }

    private void net() {
        runVoltage(kNetScoreVoltage);
    }

    private void processor() {
        runVoltage(kProcessorScoreVoltage);
    }

    private void runHold() {
        runVoltage(kHoldVoltage);
        // Don't have Phoenix Pro on this motor
        // runCurrent(kHoldCurrent);
    }

    @Override
    public CanId canIdLeader() {
        return CanId.TALONFX_ALGAE;
    }

    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return kAlgaeCurrentLimit;
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
