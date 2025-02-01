// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
    /** Creates a new Wrist. */
    private TalonFX wristMotor;
    private CANcoder wristCancoder;
    private TalonFXConfiguration wristMotorConfig;
    private CANcoderConfiguration wristCanCoderConfig;

    public Wrist() {
        wristMotor = new TalonFX(Constants.CANIDs.wristMotor, "canivore");
        wristCancoder = new CANcoder(Constants.CANIDs.wristCancoder, "canivore");
        wristMotorConfig = new TalonFXConfiguration();
        wristCanCoderConfig = new CANcoderConfiguration();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
