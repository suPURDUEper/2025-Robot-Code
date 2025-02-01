// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants;
import org.supurdueper.util.TalonFXFactory;

import static edu.wpi.first.units.Units.Amp;
import static org.supurdueper.robot2025.Constants.AlgaeScoreConstants;

public class AlgaeScore extends SubsystemBase {

    private TalonFX algaeMotor;
    private VoltageOut request = new VoltageOut(0);
    private StaticBrake stopRequest = new StaticBrake();
    private StatusSignal<Current> currentSignal;
    private LinearFilter currentFilter;
    private Debouncer currentDebouncer;
    private Current filteredCurrent; 

    private final double intakeVoltage = 12;
    private final double outtakeVoltage = -12;

    public AlgaeScore() {
        TalonFXConfiguration config = TalonFXFactory.getDefaultConfig()
            .withCurrentLimits(AlgaeScoreConstants.algaeCurrentLimit);
        algaeMotor = TalonFXFactory.createConfigTalon(CanId.TALONFX_ALGAE, config);
        currentSignal = algaeMotor.getTorqueCurrent();
        currentFilter = LinearFilter.movingAverage(AlgaeScoreConstants.hasBallCurrentFilterTaps);
        currentDebouncer = new Debouncer(AlgaeScoreConstants.hasBallCurrentDebounceTime.in(Units.Second));
    }

    @Override
    public void periodic() {
        currentSignal.refresh();
        currentFilter.calculate(currentSignal.getValueAsDouble());
    }

    // Public methods
    public Command intakeBall() {
       return Commands.runEnd(this::intake, this::stop, this).until(this::hasBall);
    }

    public Command scoreBall() {
        return Commands.runEnd(this::outtake,this::stop, this).withTimeout(AlgaeScoreConstants.scoreBallTime);
    }

    public Trigger hasBallTrigger() {
        return new Trigger(this::hasBall);
    }

    // Private methods

    private boolean hasBall() {
        return currentDebouncer.calculate(filteredCurrent.gt(AlgaeScoreConstants.hasBallCurrent));
    }

    // TODO: Separate intake and hold
    private void intake() {
        algaeMotor.setControl(request.withOutput(intakeVoltage));
    }

    // TODO: Might need different outtake for net and processor
    private void outtake() {
        algaeMotor.setControl(request.withOutput(outtakeVoltage));
    }
    
    private void stop() {
        algaeMotor.setControl(stopRequest);
    }
}
