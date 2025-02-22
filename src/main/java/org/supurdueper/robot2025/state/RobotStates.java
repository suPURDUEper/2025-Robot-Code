package org.supurdueper.robot2025.state;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import org.supurdueper.robot2025.RobotContainer;

public final class RobotStates {

    public static final Trigger sim = new Trigger(RobotBase::isSimulation);
    public static final Trigger teleop = new Trigger(DriverStation::isTeleop);
    public static final Trigger auto = new Trigger(DriverStation::isAutonomous);
    public static final Driver driver = RobotContainer.getDriver();
    public static final TestController test = RobotContainer.getTestController();

    // Autos
    @Getter
    @Setter
    private static boolean autol4 = false;

    @Getter
    @Setter
    private static boolean autoscore = false;

    @Getter
    @Setter
    private static boolean autointake = false;

    public static Trigger auto_l4 = new Trigger(() -> RobotStates.isAutol4()).and(auto);
    public static Trigger auto_score = new Trigger(() -> RobotStates.isAutoscore()).and(auto);
    public static Trigger auto_intake = new Trigger(() -> RobotStates.isAutointake()).and(auto);

    // // Information
    public static final Trigger atL1 = new Trigger(RobotContainer.getElevator()::atL1);
    public static final Trigger atL2 = new Trigger(RobotContainer.getElevator()::atL2);
    public static final Trigger atL3 = new Trigger(RobotContainer.getElevator()::atL3);
    public static final Trigger atL4 = new Trigger(RobotContainer.getElevator()::atL4);
    public static final Trigger atNet = new Trigger(RobotContainer.getElevator()::atNet);
    public static final Trigger atProcessor = new Trigger(RobotContainer.getElevator()::atProcessor);
    public static final Trigger atIntake = new Trigger(RobotContainer.getElevator()::atIntake);
    public static final Trigger atHome = new Trigger(RobotContainer.getElevator()::atHome);
    public static final Trigger hasBall = new Trigger(RobotContainer.getAlgaeScore()::hasBall);
    public static final Trigger hasCoral = new Trigger(RobotContainer.getCoralScore()::hasCoral);
    public static final Trigger atPosition = RobotContainer.getFunnelTilt()
            .isAtPosition()
            .and(RobotContainer.getWrist().isAtPosition())
            .and(RobotContainer.getElevator().isAtPosition());

    // Actions
    public static final Trigger actionScore = driver.score_rb.and(teleop);
    public static final Trigger actionIntake = driver.intake_lb.and(teleop);
    // public static final Trigger actionUnjamIntake = driver.unjam_RB.and(teleop);
    public static final Trigger actionL1 = driver.l1_A.and(teleop);
    public static final Trigger actionL2 = driver.l2_B.and(teleop);
    public static final Trigger actionL3 = driver.l3_X.and(teleop);
    public static final Trigger actionL4 = driver.l4_Y.and(teleop);
    public static final Trigger actionReef = actionL2.or(actionL3).or(actionL4);
    public static final Trigger actionNet = driver.net_fY.and(teleop);
    public static final Trigger actionProcessor = driver.processor_fX.and(teleop);
    public static final Trigger actionHome = driver.home_fA.and(teleop);
    public static final Trigger actionClimbPrep = Trigger.kFalse;
    public static final Trigger actionClimb = Trigger.kFalse;

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
