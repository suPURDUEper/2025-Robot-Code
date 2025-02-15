package org.supurdueper.robot2025.state;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.supurdueper.robot2025.RobotContainer;

public final class RobotStates {

    public static final Trigger sim = new Trigger(RobotBase::isSimulation);
    public static final Trigger teleop = new Trigger(DriverStation::isTeleop);
    public static final Trigger auto = new Trigger(DriverStation::isAutonomous);
    public static final Driver driver = RobotContainer.getDriver();

    // // Information
    // public static final Trigger hasBall = Trigger.kFalse;
    // public static final Trigger hasCoral = Trigger.kFalse;
    // public static final Trigger atL1 = Trigger.kFalse;
    // public static final Trigger atL2 = Trigger.kFalse;
    // public static final Trigger atL3 = Trigger.kFalse;
    // public static final Trigger atL4 = Trigger.kFalse;
    // public static final Trigger atNet = Trigger.kFalse;
    // public static final Trigger atProcessor = Trigger.kFalse;

    // Actions
    public static final Trigger actionScore = Trigger.kFalse;
    public static final Trigger actionIntake = driver.intake_RT.and(teleop).or(auto.and());
    public static final Trigger actionUnjamIntake = driver.unjam_RB.and(teleop).or(auto.and());
    public static final Trigger actionL1 = driver.l1_A.and(teleop).or(auto.and());
    public static final Trigger actionL2 = driver.l2_B.and(teleop).or(auto.and());
    public static final Trigger actionL3 = driver.l3_X.and(teleop).or(auto.and());
    public static final Trigger actionL4 = driver.l4_Y.and(teleop).or(auto.and());
    public static final Trigger actionReef = actionL2.or(actionL3).or(actionL4);
    public static final Trigger actionNet = Trigger.kFalse;
    public static final Trigger actionProcessor = Trigger.kFalse;
    public static final Trigger actionHome = Trigger.kFalse;
    public static final Trigger actionClimbPrep = Trigger.kFalse;
    public static final Trigger actionClimb = Trigger.kFalse;

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
