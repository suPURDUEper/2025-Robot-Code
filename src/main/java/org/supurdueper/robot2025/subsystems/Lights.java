// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.supurdueper.robot2025.state.RobotStates.actionRightAim;

import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.state.RobotStates;

public class Lights extends SubsystemBase implements SupurdueperSubsystem{
    /** Creates a new Lights. */
    private CANdle candle;

    private CANdleConfiguration candleConfig;

    public Lights() {
        candle = new CANdle(CanId.CANDLE.getDeviceNumber(), CanId.CANDLE.getBus());
        candleConfig = new CANdleConfiguration();
        candleConfig.stripType = LEDStripType.RGB;
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
        return Commands.startEnd(this::turnBlue, this::turnOff);
    }

    public Command setGreen() {
        return Commands.startEnd(this::turnGreen, this::turnOff);
    }

    public Command setRed() {
        return Commands.startEnd(this::turnRed, this::turnOff);
    }

    public Command setWhite() {
        return Commands.startEnd(this::turnWhite, this::turnOff);
    }

    public Command setGold() {
        return Commands.startEnd(this::turnGold, this::turnOff);
    }

    @Override
    public void bindCommands() {
        RobotStates.actionL1.onTrue(setGreen());
        RobotStates.hasCoral.onTrue(setRed());
        RobotStates.actionLeftAim.or(actionRightAim).onTrue(setBlue());
        RobotStates.actionScore.onTrue(setWhite());
    }
}
