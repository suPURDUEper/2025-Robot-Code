// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static edu.wpi.first.units.Units.*;
import static org.supurdueper.robot2025.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.supurdueper.lib.CurrentStallFilter;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.Robot;
import org.supurdueper.robot2025.state.RobotStates;

public class Elevator extends PositionSubsystem implements SupurdueperSubsystem {

    CurrentStallFilter homingDetector;
    PositionVoltage pidTuning = new PositionVoltage(0);
    private CANrange canRange;
    private CANrangeConfiguration canRangeConfig;

    public enum ElevatorHeight {
        L1,
        L2,
        L3,
        L4,
        Net,
        Processor,
        Intake,
        Home;
    }

    @Getter
    private ElevatorHeight currentHeightState;

    public Elevator() {
        canRange = new CANrange(CanId.CANRANGE_CORAL.getDeviceNumber(), CanId.CANRANGE_CORAL.getBus());
        canRangeConfig = new CANrangeConfiguration()
                .withToFParams(new ToFParamsConfigs()
                        .withUpdateMode(UpdateModeValue.ShortRangeUserFreq)
                        .withUpdateFrequency(50));
        canRange.getConfigurator().apply(canRangeConfig);
        configureMotors();
        homingDetector = new CurrentStallFilter(motor.getStatorCurrent(), kHomingCurrent);
        Robot.add(this);
        motor.setPosition(0);
        currentHeightState = ElevatorHeight.Home;
    }

    @Override
    public void bindCommands() {
        RobotStates.actionL1.onTrue(setHeight(ElevatorHeight.L1));
        RobotStates.actionL2.onTrue(setHeight(ElevatorHeight.L2));
        RobotStates.actionL3.onTrue(setHeight(ElevatorHeight.L3));
        RobotStates.actionL4.onTrue(setHeight(ElevatorHeight.L4));
        RobotStates.actionProcessor.onTrue(setHeight(ElevatorHeight.Processor));
        RobotStates.actionNet.onTrue(setHeight(ElevatorHeight.Net));
        RobotStates.actionIntake.onTrue(setHeight(ElevatorHeight.Intake));
        RobotStates.hasCoral.onTrue(setHeight(ElevatorHeight.Home));
        RobotStates.actionHome.onTrue(setHeight(ElevatorHeight.Home));
        RobotStates.actionScore.onFalse(
                Commands.waitSeconds(0.25).unless(this::atNet).andThen(setHeight(ElevatorHeight.Home)));
    }

    public Command setHeight(ElevatorHeight height) {
        Distance setpoint;
        switch (height) {
            case L1:
                setpoint = kL1Height;
                break;
            case L2:
                setpoint = kL2Height;
                break;
            case L3:
                setpoint = kL3Height;
                break;
            case L4:
                setpoint = kL4Height;
                break;
            case Net:
                setpoint = kNetHeight;
                break;
            case Processor:
                setpoint = kProcessorHeight;
                break;
            case Intake:
                setpoint = kIntakeHeight;
                break;
            case Home:
            default:
                setpoint = kBottomHeight;
        }
        return Commands.runOnce(() -> currentHeightState = height)
                .alongWith(goToHeight(setpoint))
                .withName("Elevator." + height.name());
    }

    public boolean atL1() {
        return currentHeightState.equals(ElevatorHeight.L1);
    }

    public boolean atL2() {
        return currentHeightState.equals(ElevatorHeight.L2);
    }

    public boolean atL3() {
        return currentHeightState.equals(ElevatorHeight.L3);
    }

    public boolean atL4() {
        return currentHeightState.equals(ElevatorHeight.L4);
    }

    public boolean atNet() {
        return currentHeightState.equals(ElevatorHeight.Net);
    }

    public boolean atProcessor() {
        return currentHeightState.equals(ElevatorHeight.Processor);
    }

    public boolean atIntake() {
        return currentHeightState.equals(ElevatorHeight.Intake);
    }

    public boolean atHome() {
        return currentHeightState.equals(ElevatorHeight.Home);
    }

    // Temporary until we figure out why magic motion isn't working
    // @Override
    // public Command goToPosition(Angle motorRotations) {
    //     return run(() -> motor.setControl(pidTuning.withPosition(motorRotations)));
    // }

    public Command goToHeight(Distance height) {
        return goToPosition(heightToMotorRotations(height));
    }

    public Command goToHeightBlocking(Distance height) {
        return goToPositionBlocking(heightToMotorRotations(height));
    }

    public void zeroMotor() {
        motor.setPosition(0);
    }

    public Command zero() {
        return runEnd(() -> runVoltage(Volts.of(-2)), this::zeroMotor)
                .until(() -> homingDetector.isStalled())
                .withName("Elevator.Zero");
    }

    @Override
    public void periodic() {
        super.periodic();
        homingDetector.periodic();
        // Log out to Glass for debugging
        DogLog.log("Elevator/Position", motorRotationToHeight(getPosition()).in(Units.Inches));
        DogLog.log(
                "Elevator/Target Position", motorRotationToHeight(getSetpoint()).in(Units.Inches));
        DogLog.log("Elevator/At Position", atPosition());
        DogLog.log("Elevator/State", currentHeightState.toString());
        DogLog.log(
                "Elevator/Distance from reef", canRange.getDistance().getValue().in(Inches));
    }

    private Distance motorRotationToHeight(Angle motorRotations) {
        return Units.Inches.of(motorRotations.in(Units.Rotation) * kInchesPerRotation);
    }

    private Angle heightToMotorRotations(Distance height) {
        return Units.Rotations.of(height.in(Units.Inches) / kInchesPerRotation);
    }

    @Override
    public Slot0Configs pidGains() {
        return new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(kp)
                .withKI(ki)
                .withKD(kd)
                .withKS(ks)
                .withKV(kv)
                .withKA(ka)
                .withKG(kg);
    }

    @Override
    public MotionMagicConfigs motionMagicConfig() {
        return new MotionMagicConfigs().withMotionMagicExpo_kA(profileKv).withMotionMagicExpo_kV(profileKa);
    }

    @Override
    public SoftwareLimitSwitchConfigs softLimitConfig() {
        return new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(heightToMotorRotations(kForwardSoftLimit))
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(heightToMotorRotations(kReverseSoftLimit))
                .withReverseSoftLimitEnable(true);
    }

    @Override
    public Angle positionTolerance() {
        return heightToMotorRotations(kPositionTolerance);
    }

    @Override
    public CanId canIdLeader() {
        return CanId.TALONFX_ELEVATOR_LEADER;
    }

    @Override
    public CanId canIdFollower() {
        return CanId.TALONFX_ELEVATOR_FOLLOWER;
    }

    @Override
    public CurrentLimitsConfigs currentLimits() {
        return new CurrentLimitsConfigs()
                .withStatorCurrentLimit(kStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(kSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);
    }

    @Override
    public boolean inverted() {
        return false;
    }

    @Override
    public boolean brakeMode() {
        return true;
    }

    @Override
    public SysIdRoutine sysIdConfig() {
        return new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        Volts.of(1).per(Second),
                        Volts.of(7),
                        null,
                        state -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motor(s).
                        this::runVoltage,
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being
                        // characterized.
                        null, // Using the CTRE SignalLogger API instead
                        // Tell SysId to make generated commands require this subsystem, suffix test
                        // state in
                        // WPILog with this subsystem's name ("shooter")
                        this));
    }

    @Override
    public boolean followerInverted() {
        return false;
    }
}
