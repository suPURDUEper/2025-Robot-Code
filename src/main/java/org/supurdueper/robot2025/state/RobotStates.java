package org.supurdueper.robot2025.state;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class RobotStates {

    public static final Trigger sim = new Trigger(RobotBase::isSimulation);

    // Information
    public static final Trigger hasBall = Trigger.kFalse;
    public static final Trigger hasCoral = Trigger.kFalse;
    public static final Trigger atL1 = Trigger.kFalse;
    public static final Trigger atL2 = Trigger.kFalse;
    public static final Trigger atL3 = Trigger.kFalse;
    public static final Trigger atL4 = Trigger.kFalse;
    public static final Trigger atNet = Trigger.kFalse;
    public static final Trigger atProcessor = Trigger.kFalse;

    // Actions
    public static final Trigger actionScore = Trigger.kFalse;
    public static final Trigger actionIntake = Trigger.kFalse;
    public static final Trigger actionL1 = Trigger.kFalse;
    public static final Trigger actionL2 = Trigger.kFalse;
    public static final Trigger actionL3 = Trigger.kFalse;
    public static final Trigger actionL4 = Trigger.kFalse;
    public static final Trigger actionReef = actionL1.or(actionL2).or(actionL3).or(actionL4);
    public static final Trigger actionNet = Trigger.kFalse;
    public static final Trigger actionProcessor = Trigger.kFalse;
    public static final Trigger actionHome = Trigger.kFalse;
    public static final Trigger actionClimbPrep = Trigger.kFalse;
    public static final Trigger actionClimb = Trigger.kFalse;

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
