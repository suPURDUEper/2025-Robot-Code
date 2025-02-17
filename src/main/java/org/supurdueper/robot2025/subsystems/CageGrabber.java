// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;

public class CageGrabber extends TalonFXSubsystem {

    private DigitalInput cageSensor;

    public CageGrabber() {
        cageSensor = new DigitalInput(Constants.DIOport.climberBreakbeam1);
        configureMotors();
    }

    private boolean hasCage() {
        return cageSensor.get();
    }

    private void grab() {
        runVoltage(Constants.CageGrabberConstants.kGrabVolrage);
    }

    public Command runForwards() {
        return Commands.runEnd(this::grab, this::stop, this);
    }

    public Command grabCage() {
        return Commands.runEnd(this::grab, this::stop, this).until(this::hasCage);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("CageGrabber/HasCage", hasCage());
    }

    @Override
    public CanId canIdLeader() {
        return CanId.TALONFX_CLIMBER_GRAB;
    }

    @Override
    public CanId canIdFollower() {
        return null;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return Constants.CageGrabberConstants.kCurrentLimit;
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
