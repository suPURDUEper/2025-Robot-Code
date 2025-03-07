// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.supurdueper.robot2025;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import org.supurdueper.lib.utils.ExpCurve;

public final class Constants {

    public static final class DriverConstants {
        public static final int kControllerPort = 0;
        public static final double kDeadzone = 0.1;
        public static final ExpCurve kLeftStickCurve = new ExpCurve(2.0, 0, 1, kDeadzone);
        public static final ExpCurve kRightStickCurve = new ExpCurve(2.0, 0, 1, kDeadzone);
        public static final ExpCurve kTriggerCurve = new ExpCurve(1, 0, 1, kDeadzone);
        public static final double kSlowModeScalor = 0.45;
        public static final double kDefaultTurnScalor = 0.75;
        public static final double kTurboModeScalor = 1;
    }

    public static final class DriveConstants {
        public static final double headingKp = 10.0;
        public static final double headingKi = 0;
        public static final double headingKd = 0.75;
    }

    public final class DIOPort {

        // Intake
        public static final int intakeBreakbeam = 5;
        // Coral score
        public static final int scoreBreakbeam1 = 0;
        public static final int scoreBreakbeam2 = -1;
        // Climber
        public static final int climberBreakbeam1 = 2;
        public static final int climberBreakbeam2 = 3;
    }

    public static final class AlgaeScoreConstants {
        public static final Current kHasBallCurrent = Amps.of(50);
        public static final CurrentLimitsConfigs kAlgaeCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);
        public static final Voltage kIntakeVoltage = Volts.of(12);
        public static final Current kHoldCurrent = Amps.of(8);
        public static final Voltage kNetScoreVoltage = Volts.of(-12);
        public static final Time kNetScoreTime = Seconds.of(0.5);
        public static final Voltage kProcessorScoreVoltage = Volts.of(-12);
        public static final Voltage kHoldVoltage = Volts.of(2);
        public static final Time kProcessorScoreTime = Seconds.of(0.5);
    }

    public final class ElevatorConstants {
        public static final double kp = 1.25;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double ks = 0.04;
        public static final double kv = 0.15;
        public static final double ka = 0.00185553096;
        public static final double kg = 0.38;
        public static final double profileKv = 0.09;
        public static final double profileKa = 0.002;
        public static final Current kStatorCurrentLimit = Amps.of(80);
        public static final Current kSupplyCurrentLimit = Amps.of(40);
        public static final double kInchesPerRotation = (12.0 / 72.0) * Math.PI * 1.744 * 2;
        public static final Distance kForwardSoftLimit = Inches.of(56.5);
        public static final Distance kReverseSoftLimit = Inches.of(0);
        public static final Distance kPositionTolerance = Inches.of(0.5);
        public static final Current kHomingCurrent = Amps.of(13);
        public static final Distance kBottomHeight = Inches.of(0);
        public static final Distance kIntakeHeight = Inches.of(0);
        public static final Distance kL1Height = Inches.of(1);
        public static final Distance kL2Height = Inches.of(10.8);
        public static final Distance kL3Height = Inches.of(26.75);
        public static final Distance kL4Height = Inches.of(56);
        public static final Distance kProcessorHeight = Inches.of(1);
        public static final Distance kNetHeight = Inches.of(55);
    }

    public final class WristConstants {
        public static final double kp = 200.0;
        public static final double ki = 0;
        public static final double kd = 8.0;
        public static final double ks = 0;
        public static final double kv = 16.8;
        public static final double ka = 0;
        public static final double kg = 0.12;
        public static final double profileKv = 0;
        public static final double profileKa = 0;
        public static final Current kStatorCurrentLimit = Amps.of(80);
        public static final Angle kForwardSoftLimit = Degrees.of(115);

        public static final Angle kReverseSoftLimit = Degrees.of(43);
        public static final Angle kPositionTolerance = Degrees.of(1);
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(20).withStatorCurrentLimitEnable(true);
        public static final double kDegreesPerRotation = 0;
        public static final Angle kCancoderMagnetOffset = Degrees.of(190);
        public static final Angle kHomeAngle = Degrees.of(81);
        public static final Angle kIntakeAngle = Degrees.of(75.75);
        public static final Angle kL1Angle = Degrees.of(66.78);
        public static final Angle kL2Angle = Degrees.of(69);
        public static final Angle kL3Angle = kL2Angle;
        public static final Angle kL4Angle = Degrees.of(43);
        public static final Angle kProcessorAngle = Degrees.of(43);
        public static final Angle kNetAngle = Degrees.of(115);
        public static final Angle kClimbPrep = Degrees.of(43);
    }

    public final class ClimberConstants {
        public static final double kp = 15.0;
        public static final double ki = 0;
        public static final double kd = 0.0;
        public static final double ks = 0;
        public static final double kv = 0.0;
        public static final double ka = 0;
        public static final double kg = 0.0;
        public static final double kHome = 0.0;
        public static final double kRetract = -12;
        public static final double kClearFunnel = 21.0;
        public static final double kClimb = 48.8;
        public static final double kPositionTolerance = 1.0;
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(120).withStatorCurrentLimitEnable(true);
        public static final double kEngagePercent = 0.75;
        public static final double kUnengagePercent = 0;
    }

    public final class CageGrabberConstants {
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);
        public static final Voltage kGrabVolrage = Volts.of(12);
    }

    public final class FunnelConstants {
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true);
        public static final Voltage kIntakeVoltage = Volts.of(7);
        public static final Voltage kUnjamVoltage = Volts.of(-4);
    }

    public final class CoralScoreConstants {
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true);
        public static final Voltage scoreVoltage = Volts.of(12);
        public static final Voltage scoreL1Voltage = Volts.of(8);
        public static final Voltage scoreL2L3Voltage = Volts.of(4);
        public static final Voltage scoreL4Voltage = Volts.of(12);
        public static final Voltage backupVoltage = Volts.of(-2);
        public static final Voltage unJamVoltage = Volts.of(-6);
        public static final DebounceType kDebounceTime = null;
        public static final Voltage loadVoltage = Volts.of(5);
    }

    public final class FunnelTiltConstants {
        public static final double kp = 75;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
        public static final double kg = 0;
        public static final double profileKv = 0;
        public static final double profileKa = 0;
        public static final Angle kForwardSoftLimit = Degrees.of(90);
        public static final Angle kReverseSoftLimit = Degrees.of(-97);
        public static final Angle kPositionTolerance = Degrees.of(1);
        public static final CurrentLimitsConfigs kCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true);
        public static final double kSensorToMechanismRatio = 27.0 / 1.0 * 44.0 / 14.0;
        public static final double kAbsEncoderRatio = 80.0 / 48.0;
        public static final Angle kAbsEncoderOffset = Degrees.of(101.485065);
        public static final Angle kIntakePosition = Degrees.of(-20);
        public static final Angle kStartPosition = Degrees.of(-90);
        public static final Angle kL1Position = Degrees.of(45);
        public static final Angle kClimbPosition = Degrees.of(90);
        public static final Angle kCancoderMagnetOffset = Rotations.of(0.361572);
    }

    public static boolean tuningMode = true;
}
