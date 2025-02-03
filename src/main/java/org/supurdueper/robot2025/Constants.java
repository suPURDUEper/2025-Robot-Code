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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import org.supurdueper.lib.utils.ExpCurve;

public final class Constants {

    public static final class DriverConstants {
        public static final int kControllerPort = 0;
        public static final double kDeadzone = 0.001;
        public static final ExpCurve kLeftStickCurve = new ExpCurve(2.0, 0, 6, kDeadzone);
        public static final ExpCurve kRightStickCurve = new ExpCurve(2.0, 0, 12, kDeadzone);
        public static final ExpCurve kTriggerCurve = new ExpCurve(1, 0, 1, kDeadzone);
        public static final double kSlowModeScalor = 0.45;
        public static final double kDefaultTurnScalor = 0.75;
        public static final double kTurboModeScalor = 1;
    }

    public final class DIOport {

        // Intake
        public static final int intakeBreakbeam = 0;
        // Coral score
        public static final int scoreBreakbeam1 = 0;
        public static final int scoreBreakbeam2 = 0;
        // Climber
        public static final int climberBreakbeam1 = 0;
        public static final int climberBreakbeam2 = 0;
    }

    public static final class AlgaeScoreConstants {
        public static final Current kHasBallCurrent = Amps.of(30);
        public static final CurrentLimitsConfigs kAlgaeCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(20).withStatorCurrentLimitEnable(true);
        public static final Voltage kIntakeVoltage = Volts.of(12);
        public static final Current kHoldCurrent = Amps.of(8);
        public static final Voltage kNetScoreVoltage = Volts.of(-12);
        public static final Time kNetScoreTime = Seconds.of(0.5);
        public static final Voltage kProcessorScoreVoltage = Volts.of(-12);
        public static final Time kProcessorScoreTime = Seconds.of(0.5);
    }

    public final class ElevatorConstants {
        public static final double kp = 0;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
        public static final double kg = 0;
        public static final double profileKv = 0;
        public static final double profileKa = 0;
        public static final Current kStatorCurrentLimit = Amps.of(80);
        public static final Current kSupplyCurrentLimit = Amps.of(40);
        public static final double kInchesPerRotation = 72 / 12 * Math.PI * 1.744;
        public static final Distance kForwardSoftLimit = Inches.of(55);
        public static final Distance kReverseSoftLimit = Inches.of(0);
        public static final Distance kPositionTolerance = Inches.of(0.5);
        public static final Current kHomingCurrent = Amps.of(13);
    }

    public final class WristConstants {
        public static final double kp = 0;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double ks = 0;
        public static final double kv = 0;
        public static final double ka = 0;
        public static final double kg = 0;
        public static final double profileKv = 0;
        public static final double profileKa = 0;
        public static final Current kStatorCurrentLimit = Amps.of(80);
        public static final Angle kForwardSoftLimit = Degrees.of(55);
        public static final Angle kReverseSoftLimit = Degrees.of(0);
        public static final Angle kPositionTolerance = Degrees.of(0.25);
    }

    public final class ClimberConstants {}

    public final class FunnelConstants {
        public static final CurrentLimitsConfigs funnelCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(20).withStatorCurrentLimitEnable(true);
    }

    public final class CoralScoreConstants {}

    public static boolean tuningMode = true;
}
