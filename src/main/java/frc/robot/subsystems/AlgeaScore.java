// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgeaScore extends SubsystemBase {
    /** Creates a new AlgaeIntake. */
    private DigitalInput breakbeam;

    private TalonFX algeaMotor;
    private TalonFX pivotMotor;
    private TalonFXConfiguration algeaConfig;

    public AlgeaScore() {
        algeaMotor = new TalonFX(Constants.CANIDs.algeaMotor, "canivore");
        pivotMotor = new TalonFX(Constants.CANIDs.pivotMotor, "canivore");
        algeaConfig = new TalonFXConfiguration();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
