// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    private TalonFX intakeMotor;
    private TalonFXConfiguration intakeConfigs;
    private DigitalInput breakbeam;

    /** Creates a new CoralIntake. */
    public CoralIntake() {
        intakeMotor = new TalonFX(Constants.CANIDs.intakeMotor, "canivore");
        intakeConfigs = new TalonFXConfiguration();
        breakbeam = new DigitalInput(Constants.DIOport.intakeBreakbeam);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
