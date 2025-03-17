// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.state.RobotStates;

public class Lights extends SubsystemBase implements SupurdueperSubsystem {
    /** Creates a new Lights. */
    private CANdle candle;

    private CANdleConfiguration candleConfig;

    public Lights() {
        super();
        candle = new CANdle(CanId.CANDLE.getDeviceNumber());
        candleConfig = new CANdleConfiguration();
        candleConfig.stripType = LEDStripType.GRB;
        candleConfig.brightnessScalar = 0.5;
        candle.configAllSettings(candleConfig);
        Robot.add(this);
    }

    private void turnBlue() {
        candle.setLEDs(0, 0, 255);
    }

    private void turnRed() {
        candle.setLEDs(255, 0, 0);
    }

    private void turnGreen() {
        candle.setLEDs(0, 255, 0);
    }

    private void turnGold() {
        candle.setLEDs(217, 160, 15);
    }

    private void turnWhite() {
        candle.setLEDs(255, 255, 255);
    }

    private void turnOff() {
        candle.setLEDs(0, 0, 0);
    }

    public Command setBlue() {
        return run(this::turnBlue);
    }

    public Command setGreen() {
        return run(this::turnGreen);
    }

    public Command setRed() {
        return run(this::turnRed);
    }

    public Command setWhite() {
        return run(this::turnWhite);
    }

    public Command setGold() {
        return run(this::turnGold);
    }

    @Override
    public void bindCommands() {
        RobotStates.atL1.whileTrue(setGreen());
        RobotStates.atL1.whileFalse(run(this::turnOff));
        RobotStates.actionAim.onTrue(setRed());
        RobotStates.actionScore.onTrue(run(this::turnOff));
        RobotStates.actionLeftAim.onFalse(run(this::turnOff));
        RobotStates.actionRightAim.onFalse(run(this::turnOff));
        RobotStates.isAimed.whileFalse(setRed());
        RobotStates.isAimed.whileTrue(setBlue());
        // RobotStates.actionLeftAim.or(actionRightAim).onTrue(setBlue());
        // RobotStates.actionScore.onTrue(setWhite());
        RobotStates.teleop.onTrue(setGold());
    }
}
