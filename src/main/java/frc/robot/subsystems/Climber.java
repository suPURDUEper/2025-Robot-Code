// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private TalonFX grabMotor;
    private TalonFX pulleyMotor1;
    private TalonFX pulleyMotor2;
    private TalonFXConfiguration climbConfig;
    private DigitalInput breakbeam1;
    private DigitalInput breakbeam2;


    public Climber() {
        grabMotor = new TalonFX(Constants.CANIDs.grabMotor, "canivore");
        pulleyMotor1 = new TalonFX(Constants.CANIDs.pulleyMotor1, "canivore");
        pulleyMotor2 = new TalonFX(Constants.CANIDs.pulleyMotor2, "canivore");
        breakbeam1 = new DigitalInput(0);
        breakbeam2 = new DigitalInput(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
