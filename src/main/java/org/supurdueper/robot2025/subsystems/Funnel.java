// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.supurdueper.lib.TalonFXFactory;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;
import org.supurdueper.robot2025.Constants.FunnelConstants;

public class Funnel extends SubsystemBase {
    private TalonFX funnelMotor;
    private TalonFX tiltMotor;
    private TalonFXConfiguration intakeConfigs;
    private StaticBrake stopRequest = new StaticBrake();
    private DigitalInput breakbeam;

    /** Creates a new CoralIntake. */
    public Funnel() {
        TalonFXConfiguration config =
                TalonFXFactory.getDefaultConfig().withCurrentLimits(FunnelConstants.funnelCurrentLimit);
        funnelMotor = TalonFXFactory.createDefaultTalon(CanId.TALONFX_FUNNEL);
        tiltMotor = TalonFXFactory.createDefaultTalon(CanId.TALONFX_FUNNEL_TILT);
        intakeConfigs = new TalonFXConfiguration();
        breakbeam = new DigitalInput(Constants.DIOport.intakeBreakbeam);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command runIntake() {
        return Commands.runEnd(this::intake, this::stopIntake);
    }

    private void intake() {}

    private void stopIntake() {}

    public boolean hasCoral() {
        return breakbeam.get();
    }
}
