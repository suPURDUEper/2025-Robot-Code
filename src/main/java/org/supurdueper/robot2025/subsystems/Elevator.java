// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.supurdueper.robot2025.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.supurdueper.lib.CurrentStallFilter;
import org.supurdueper.lib.subsystems.PositionSubsystem;
import org.supurdueper.lib.subsystems.SupurdueperSubsystem;
import org.supurdueper.robot2025.CanId;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.RobotStates;
import org.supurdueper.robot2025.state.TestController;

public class Elevator extends PositionSubsystem implements SupurdueperSubsystem {

    CurrentStallFilter homingDetector;

    public Elevator() {
        configureMotors();
        homingDetector = new CurrentStallFilter(motor.getStatorCurrent(), kHomingCurrent);
    }

    public Command l1() {
        return goToHeight(kL1Height).withName("Elevator.L1");
    }

    public Command l2() {
        return goToHeight(kL2Height).withName("Elevator.L2");
    }

    public Command l3() {
        return goToHeight(kL3Height).withName("Elevator.L3");
    }

    public Command l4() {
        return goToHeight(kL4Height).withName("Elevator.L4");
    }

    public Command net() {
        return goToHeight(kNetHeight).withName("Elevator.Net");
    }

    public Command processor() {
        return goToHeight(kProcessorHeight).withName("Elevator.Processor");
    }

    public Command home() {
        return goToHeight(kBottomHeight).withName("Elevator.Bottom");
    }

    public Command goToHeight(Distance height) {
        return goToPosition(heightToMotorRotations(height));
    }

    public Command goToHeightBlocking(Distance height) {
        return goToPositionBlocking(heightToMotorRotations(height));
    }

    public Command zero() {
        return Commands.runEnd(() -> runVoltage(Volts.of(-2)), () -> motor.setPosition(0), this)
                .until(() -> homingDetector.isStalled())
                .withName("Elevator.Zero");
    }

    public Command setVoltage(DoubleSupplier voltage) {
        return Commands.run(() -> runVoltage(Volts.of(voltage.getAsDouble())), this);
    }

    @Override
    public void bindCommands() {
        RobotStates.actionL1.onTrue(l1());
        RobotStates.actionL2.onTrue(l2());
        RobotStates.actionL3.onTrue(l3());
        RobotStates.actionL4.onTrue(l4());
        RobotStates.actionProcessor.onTrue(processor());
        RobotStates.actionNet.onTrue(net());
        RobotStates.actionScore.onFalse(home());
        TestController testController = RobotContainer.getTestController();
        RobotStates.actionManualElevator.onTrue(setVoltage(testController::getManualElevator));
    }

    @Override
    public void periodic() {
        super.periodic();
        homingDetector.periodic();
        // Log out to Glass for debugging
        double elevatorPosition = motorRotationToHeight(getPosition()).in(Units.Inches);
        double elevatorSetpoint = motorRotationToHeight(getSetpoint()).in(Units.Inches);
        SmartDashboard.putNumber("Elevator/Position (Motor)", elevatorPosition);
        SmartDashboard.putNumber("Elevator/Target Position", elevatorSetpoint);
        SmartDashboard.putBoolean("Elevator/At Position", atPosition());
    }

    private Distance motorRotationToHeight(Angle motorRotations) {
        return Units.Inches.of(motorRotations.in(Units.Rotation) * kInchesPerRotation);
    }

    private Angle heightToMotorRotations(Distance height) {
        return Units.Rotations.of(height.in(Units.Inches) / kInchesPerRotation);
    }

    /**
     * ************************************************************************ PositionSubsystem setup
     * ************************************************************************
     */
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'followerInverted'");
    }
}
