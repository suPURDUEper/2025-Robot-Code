// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static org.supurdueper.robot2025.Constants.AlgaeScoreConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.utils.CurrentStallFilter;

public class AlgaeScore extends TalonFXSubsystem {

    private CurrentStallFilter ballDetector;

    public AlgaeScore() {
        configureMotors();
        ballDetector = new CurrentStallFilter(motor.getStatorCurrent(), kHasBallCurrent);
    }

    @Override
    public void periodic() {
        super.periodic();
        ballDetector.periodic();
    }

    // Public methods
    public Command intake() {
        return Commands.runEnd(this::runIntake, this::hold, this).until(this::hasBall);
    }

    public Command scoreNet() {
        return Commands.runEnd(this::net, this::stop, this).withTimeout(netScoreTime);
    }

    public Command scoreProcessor() {
        return Commands.runEnd(this::processor, this::stop, this).withTimeout(kNetScoreTime);
    }

    public Trigger hasBallTrigger() {
        return ballDetector.stallTrigger();
    }

    // Private methods
    private boolean hasBall() {
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
        runCurrent(kHoldCurrent);
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
        return new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);
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
