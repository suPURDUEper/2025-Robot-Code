// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.RobotStates;

public class CoralScore extends TalonFXSubsystem implements SupurdueperSubsystem {

    private DigitalInput coralScoreBack;
    private DigitalInput coralScoreFront;

    public CoralScore() {
        coralScoreBack = new DigitalInput(Constants.DIOPort.scoreBreakbeamBack);
        coralScoreFront = new DigitalInput(Constants.DIOPort.scoreBreakbeamFront);

        configureMotors();
        Robot.add(this);
    }

    public boolean hasCoral() {
        return !coralScoreBack.get() || !coralScoreFront.get();
    }

    public boolean scoredCoral() {
        return coralScoreFront.get();
    }

    public Command loadCoral() {
        return runEnd(this::load, this::stop)
                .until(this::hasCoral)
                .andThen(runEnd(this::slowLoad, this::stop).until(() -> !coralScoreFront.get()))
                .withName("CoralScore.LoadCoral");
    }

    private void load() {
        runVoltage(Constants.CoralScoreConstants.loadVoltage);
    }

    private void slowLoad() {
        runVoltage(Constants.CoralScoreConstants.slowLoadVoltage);
    }

    public Command l4() {
        return runEnd(this::scoreL4, this::stop).withName("CoralScore.L4");
    }

    private void scoreL4() {
        runVoltage(Constants.CoralScoreConstants.scoreL4Voltage);
    }

    public Command l2L3() {
        return runEnd(this::scoreL2L3, this::stop).withName("CoralScore.L2L3");
    }

    private void scoreL2L3() {
        runVoltage(Constants.CoralScoreConstants.scoreL2L3Voltage);
    }

    public Command l1() {
        return runEnd(this::scoreL1, this::stop).withName("CoralScore.L1");
    }

    private void scoreL1() {
        runVoltage(Constants.CoralScoreConstants.scoreL1Voltage);
    }

    public Command unJam() {
        return runEnd(this::runBackwards, this::stop).withName("CoralScore.UnJam");
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
        RobotStates.actionUnjam.whileTrue(unJam());
        new Trigger(this::hasCoral)
                .onTrue(Commands.runEnd(() -> RobotContainer.getDriver().rumble(1, 1), () -> RobotContainer.getDriver()
                                .rumble(0, 0))
                        .withTimeout(1.0));
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("CoralScore/BreakBeam1", coralScoreBack.get());
        DogLog.log("CoralScore/BreakBeam2", coralScoreFront.get());
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
