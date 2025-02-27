package org.supurdueper.lib.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.supurdueper.lib.TalonFXFactory;
import org.supurdueper.robot2025.CanId;

public abstract class TalonFXSubsystem extends SubsystemBase {

    public VoltageOut voltageRequest = new VoltageOut(0);
    public TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
    public NeutralOut stopRequest = new NeutralOut();
    public TalonFX motor;
    public TalonFX followerMotor;
    public TalonFXConfiguration config;

    public TalonFXSubsystem() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(inverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
                .withNeutralMode(brakeMode() ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        config = TalonFXFactory.getDefaultConfig()
                .withCurrentLimits(currentLimits())
                .withMotorOutput(motorOutputConfigs);
    }

    @Override
    public void periodic() {
        super.periodic();
        DogLog.log(getClass().getSimpleName() + "/Command", getCurrentCommand().getName());
    }

    public Command setVoltage(DoubleSupplier voltage) {
        return run(() -> runVoltage(Volts.of(voltage.getAsDouble())));
    }

    protected void runVoltage(Voltage volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    protected void runCurrent(Current amps) {
        motor.setControl(torqueRequest.withOutput(amps));
    }

    protected void stop() {
        motor.setControl(stopRequest);
    }

    protected void configureMotors() {
        motor = TalonFXFactory.createConfigTalon(canIdLeader(), config);
        if (canIdFollower() != null) {
            followerMotor = TalonFXFactory.createPermanentFollowerTalon(canIdFollower(), motor, followerInverted());
        }
    }

    public abstract CanId canIdLeader();

    public abstract CanId canIdFollower();

    public abstract boolean followerInverted();

    public abstract CurrentLimitsConfigs currentLimits();

    public abstract boolean inverted();

    public abstract boolean brakeMode();
}
