// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Constants.ElevatorConstants;
import org.supurdueper.util.LoggedTunableNumber;
import org.supurdueper.util.TalonFXFactory;

public class Elevator extends SubsystemBase {

    // Tunable numbers
    private static final LoggedTunableNumber kp = new LoggedTunableNumber("Elevator/Kp");
    private static final LoggedTunableNumber ki = new LoggedTunableNumber("Elevator/Ki");
    private static final LoggedTunableNumber kd = new LoggedTunableNumber("Elevator/Kd");
    private static final LoggedTunableNumber ks = new LoggedTunableNumber("Elevator/Ks");
    private static final LoggedTunableNumber kv = new LoggedTunableNumber("Elevator/Kv");
    private static final LoggedTunableNumber ka = new LoggedTunableNumber("Elevator/Ka");
    private static final LoggedTunableNumber kg = new LoggedTunableNumber("Elevator/Kg");
    private static final LoggedTunableNumber profileKv = new LoggedTunableNumber("Elevator/profileKv");
    private static final LoggedTunableNumber profileKa = new LoggedTunableNumber("Elevator/profileKa");
    private static final List<LoggedTunableNumber> pidGains = new ArrayList<>();

    // Control requests
    private final StaticBrake stopRequest = new StaticBrake();
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);

    private TalonFX elevatorLeader;
    private TalonFX elevatorFollower;
    private TalonFXConfiguration elevatorConfig;
    private StatusSignal<Angle> elevatorPosition;
    private Distance setpoint;

    static {
        kp.initDefault(ElevatorConstants.kp);
        ki.initDefault(ElevatorConstants.ki);
        kd.initDefault(ElevatorConstants.kd);
        ks.initDefault(ElevatorConstants.kd);
        kv.initDefault(ElevatorConstants.kv);
        ka.initDefault(ElevatorConstants.ka);
        kg.initDefault(ElevatorConstants.kg);
        profileKv.initDefault(ElevatorConstants.profileKv);
        profileKa.initDefault(ElevatorConstants.profileKa);
        pidGains.addAll(List.of(kp, ki, kd, ks, kv, ka, kg, profileKa, profileKv));
    }

    public Elevator() {
        elevatorConfig = configureTalonFx(TalonFXFactory.getDefaultConfig());
        elevatorLeader = TalonFXFactory.createConfigTalon(CanId.TALONFX_ELEVATOR_LEADER, elevatorConfig);
        elevatorFollower =
                TalonFXFactory.createPermanentFollowerTalon(CanId.TALONFX_ELEVATOR_FOLLOWER, elevatorLeader, false);
    }

    // SysID Setup
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                    Volts.of(1).per(Second.of(1).baseUnit()),
                    Volts.of(1),
                    null,
                    state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motor(s).
                    this::setVoltage,
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    null, // Using the CTRE SignalLogger API instead
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("shooter")
                    this));

    private void setVoltage(Voltage voltage) {
        elevatorLeader.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void periodic() {
        for (LoggedTunableNumber gain : pidGains) {
            if (gain.hasChanged(hashCode())) {
                // Send new PID gains to talon
                Slot0Configs slot0config = new Slot0Configs()
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKP(kp.get())
                        .withKI(ki.get())
                        .withKD(kd.get())
                        .withKS(ks.get())
                        .withKV(kv.get())
                        .withKA(ka.get())
                        .withKG(kg.get());
                MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                        .withMotionMagicExpo_kA(profileKa.get())
                        .withMotionMagicExpo_kV(profileKv.get());
                elevatorLeader
                        .getConfigurator()
                        .apply(elevatorConfig.withSlot0(slot0config).withMotionMagic(motionMagicConfigs));
                break;
            }
        }
        // Log out to Glass for debugging
        double elevatorPosition = getPosition().in(Units.Inches);
        double elevatorSetpoint = getSetpoint().in(Units.Inches);
        SmartDashboard.putNumber("Elevator/Position (Motor)", elevatorPosition);
        SmartDashboard.putNumber("Elevator/Target Position", elevatorSetpoint);
        SmartDashboard.putBoolean("Elevator/At Position", isAtPosition());
    }

    private Distance getPosition() {
        return motorRotationToDistance(elevatorLeader.getPosition().getValue());
    }

    private Distance getSetpoint() {
        return motorRotationToDistance(
                Units.Rotations.of(elevatorLeader.getClosedLoopReference(true).getValue()));
    }

    public boolean isAtPosition() {
        return (setpoint.minus(getPosition())).lt(ElevatorConstants.kPositionTolerance)
                && elevatorLeader.getVelocity().getValueAsDouble() < 0.1;
    }

    private Distance motorRotationToDistance(Angle motorRotations) {
        return Units.Meters.of(motorRotations.in(Units.Rotation) * ElevatorConstants.kMetersPerRotation);
    }

    private TalonFXConfiguration configureTalonFx(TalonFXConfiguration config) {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);
        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ElevatorConstants.kStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true);
        Slot0Configs slot0config = new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(kp.get())
                .withKI(ki.get())
                .withKD(kd.get())
                .withKS(ks.get())
                .withKV(kv.get())
                .withKA(ka.get())
                .withKG(kg.get());
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicExpo_kA(profileKv.get())
                .withMotionMagicExpo_kV(profileKa.get());
        SoftwareLimitSwitchConfigs softlimitConfig = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(ElevatorConstants.kForwardSoftLimit)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ElevatorConstants.kReverseSoftLimit)
                .withReverseSoftLimitEnable(true);
        return config.withMotorOutput(motorOutputConfigs)
                .withSoftwareLimitSwitch(softlimitConfig)
                .withSlot0(slot0config)
                .withCurrentLimits(currentConfig)
                .withMotionMagic(motionMagicConfigs);
    }
}
