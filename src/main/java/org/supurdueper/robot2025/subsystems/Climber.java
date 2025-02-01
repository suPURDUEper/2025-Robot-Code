// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.supurdueper.robot2025.CanId;
import org.supurdueper.util.TalonFXFactory;

public class Climber extends SubsystemBase {
    private TalonFX grabMotor;
    private TalonFX liftMotorLeader;
    private TalonFX liftMotorFollower;
    private DigitalInput breakbeam1;
    private DigitalInput breakbeam2;

    public Climber() {
        grabMotor = TalonFXFactory.createDefaultTalon(CanId.TALONFX_CLIMBER_GRAB);
        liftMotorLeader = TalonFXFactory.createDefaultTalon(CanId.TALONFX_CLIMBER_LEADER);
        liftMotorFollower = TalonFXFactory.createPermanentFollowerTalon(CanId.TALONFX_CLIMBER_FOLLOWER,liftMotorLeader,false);
        breakbeam1 = new DigitalInput(0);
        breakbeam2 = new DigitalInput(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
