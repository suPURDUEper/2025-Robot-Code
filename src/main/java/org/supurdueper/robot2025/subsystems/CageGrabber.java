// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.lib.subsystems.TalonFXSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.state.RobotStates;

public class CageGrabber extends TalonFXSubsystem implements SupurdueperSubsystem {

    private DigitalInput cageSensorLeft;
    private DigitalInput cageSensorRight;


    public CageGrabber() {
        cageSensorLeft = new DigitalInput(Constants.DIOPort.climberBreakbeam1);
        cageSensorRight = new DigitalInput(Constants.DIOPort.climberBreakbeam2);
        configureMotors();
        Robot.add(this);
    }

    @Override
    public void bindCommands() {
        RobotStates.actionClimbPrep.onTrue(grabCage());
    }

    public boolean hasCage() {
        return cageSensorLeft.get() && cageSensorRight.get();
    }

    private void grab() {
        runVoltage(Constants.CageGrabberConstants.kGrabVolrage);
    }

    public Command runForwards() {
        return runEnd(this::grab, this::stop);
    }

    public Command grabCage() {
        return runEnd(this::grab, this::stop).until(this::hasCage).withName("CageGrabber.GrabCage");
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log("CageGrabber/HasCageLeft", cageSensorLeft.get());
        DogLog.log("CageGrabber/HasCageRight", cageSensorRight.get());
        DogLog.log("CageGrabber/HasCage", hasCage());
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
