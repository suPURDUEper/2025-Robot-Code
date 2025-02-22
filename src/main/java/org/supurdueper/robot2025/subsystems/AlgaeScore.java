// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static org.supurdueper.robot2025.Constants.AlgaeScoreConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import org.supurdueper.lib.CurrentStallFilter;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.state.RobotStates;

public class AlgaeScore extends TalonFXSubsystem implements SupurdueperSubsystem {

    private CurrentStallFilter ballDetector;

    public AlgaeScore() {
        configureMotors();
        ballDetector = new CurrentStallFilter(motor.getStatorCurrent(), kHasBallCurrent);
        Robot.add(this);
    }

    @Override
    public void bindCommands() {
        RobotStates.atL2.or(RobotStates.atL3).whileTrue(intake());
        RobotStates.actionScore.and(RobotStates.atNet).onTrue(scoreNet());
        RobotStates.actionScore.and(RobotStates.atProcessor).onTrue(scoreProcessor());
    }

    @Override
    public void periodic() {
        super.periodic();
        ballDetector.periodic();
        DogLog.log("AlgaeScore/HasBall", hasBall());
    }

    // Public methods
    public Command intake() {
        return new FunctionalCommand(
                () -> {},
                this::runIntake,
                interrupted -> {
                    if (interrupted) {
                        stop();
                    } else {
                        hold();
                    }
                },
                this::hasBall,
                this);
    }

    public Command scoreNet() {
        return runEnd(this::net, this::stop).withTimeout(kNetScoreTime);
    }

    public Command scoreProcessor() {
        return runEnd(this::processor, this::stop).withTimeout(kProcessorScoreTime);
    }

    // Private methods
    public boolean hasBall() {
        return ballDetector.isStalled();
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

    private void hold() {
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
