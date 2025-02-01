// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private TalonFX intakeMotor;
    private TalonFXConfiguration intakeConfigs;
    private DigitalInput breakbeam;
    private boolean elasticTest1;
    private boolean elasticTest2;

    /** Creates a new CoralIntake. */
    public CoralIntake() {
        elasticTest1 = true;
        elasticTest1 = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Coral/Elastic Test 1 (should be true)", elasticTest1);
        SmartDashboard.putBoolean("Coral/Elastic Test 2 (should be false)", elasticTest2);
    }
}
