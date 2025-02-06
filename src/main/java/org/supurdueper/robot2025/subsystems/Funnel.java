// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2025.CanId;

public class Funnel extends TalonFXSubsystem {

    /** Creates a new CoralIntake. */
    public Funnel() {
        configureMotors();
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public Command intake() {
        return Commands.run(() -> runVoltage(Volts.of(12)));
    }

    public Command unjam() {
        return Commands.run(() -> runVoltage(Volts.of(-10)));
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'currentLimits'");
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
