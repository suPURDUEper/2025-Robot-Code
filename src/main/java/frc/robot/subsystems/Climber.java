// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private TalonFX grabMotor1;
  private TalonFX grabMotor2;
  private TalonFX pivotMotor;
  
  public Climber() {

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
