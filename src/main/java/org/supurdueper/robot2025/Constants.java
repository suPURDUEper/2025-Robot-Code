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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
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
        public static final String hasBallCurrentDebounceTime = null;
        public static final Time netScoreTime = null;
    }

    public final class ElevatorConstants {
        public static double kp;
        public static double ki;
        public static double kd;
        public static double ks;
        public static double kv;
        public static double ka;
        public static double kg;
        public static double profileKv;
        public static double profileKa;
        public static Current kStatorCurrentLimit;
        public static double kMetersPerRotation;
        public static Distance kForwardSoftLimit;
        public static Distance kReverseSoftLimit;
        public static Distance kPositionTolerance;
        public static Current kHomingCurrent;
    }

    public final class WristConstants {}

    public final class ClimberConstants {}

    public final class FunnelConstants {
        public static final CurrentLimitsConfigs funnelCurrentLimit =
                new CurrentLimitsConfigs().withStatorCurrentLimit(20).withStatorCurrentLimitEnable(true);
    }

    public final class CoralScoreConstants {}

    public static boolean tuningMode;
}
