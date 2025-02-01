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

package frc.robot;

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

    public final class CANIDs {

            //Motors

        //Elevator
        public static final int elevatorMotor1 = 0;
        public static final int elevatorMotor2 = 0;
        //Coral scoring
        public static final int coralScoreMotor = 0;
        //Wrist
        public static final int wristMotor = 0;
        //Algea
        public static final int algeaMotor = 0;
        //Intake
        public static final int intakeMotor = 0;
        public static final int pivotMotor = 0;
        //Climber
        public static final int grabMotor = 0;
        public static final int pulleyMotor1 = 0;
        public static final int pulleyMotor2 = 0;


            //Cancoders
            
        public static final int wristCancoder = 0;
    

    }

    public final class DIOport {

        //Intake
        public static final int intakeBreakbeam = 0;
        //Coral score
        public static final int scoreBreakbeam1 = 0;
        public static final int scoreBreakbeam2 = 0;
        //Climber
        public static final int climberBreakbeam1 = 0;
        public static final int climberBreakbeam2 = 0;

    }

    public final class AlgeaScoreConstants {

    }

    
    public final class ElevatorConstants {
        
    }

    
    public final class WristConstants {
        
    }

    
    public final class ClimberConstants {
        
    }

    
    public final class CoralIntakeConstants {
        
    }

    public final class CoralScoreConstants {

    }
}
