// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.supurdueper.robot2025.Constants;

public class CoralScore extends SubsystemBase {
    /** Creates a new EndEffector. */
    private TalonFX coralScoreMotor;

    private TalonFXConfiguration coralScoreConfig;
    private DigitalInput breakbeam1;
    private DigitalInput breakbeam2;

    public CoralScore() {
        coralScoreConfig = new TalonFXConfiguration();
        breakbeam1 = new DigitalInput(Constants.DIOport.scoreBreakbeam1);
        breakbeam2 = new DigitalInput(Constants.DIOport.scoreBreakbeam2);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
