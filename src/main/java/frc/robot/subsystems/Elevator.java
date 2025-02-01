// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    /** Creates a new Elevator. */
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;


    private TalonFXConfiguration elevatorConfig;

    public Elevator() {
        elevatorMotor1 = new TalonFX(Constants.CANIDs.elevatorMotor1, "canivore");
        elevatorMotor2 = new TalonFX(Constants.CANIDs.elevatorMotor2, "canivore");
        elevatorConfig = new TalonFXConfiguration();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
