// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private CANdle candle;
  public Lights() {
    candle = new CANdle(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
