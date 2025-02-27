package org.supurdueper.robot2025.state;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import org.supurdueper.robot2025.RobotContainer;

public final class RobotStates {

    public static final Trigger sim = new Trigger(RobotBase::isSimulation);
    public static final Trigger teleop = RobotModeTriggers.teleop();
    public static final Trigger auto = RobotModeTriggers.autonomous();
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

    @Getter
    @Setter
    private static boolean autol3 = false;

    @Getter
    @Setter
    private static boolean autol2 = false;

    public static final Trigger auto_l4 = new Trigger(RobotStates::isAutol4).and(auto);
    public static final Trigger auto_l3 = new Trigger(RobotStates::isAutol3).and(auto);
    public static final Trigger auto_l2 = new Trigger(RobotStates::isAutol2).and(auto);
    public static final Trigger auto_score = new Trigger(RobotStates::isAutoscore).and(auto);
    public static final Trigger auto_intake = new Trigger(RobotStates::isAutointake).and(auto);

    // // Information
    public static final Trigger atL1 = new Trigger(RobotContainer.getElevator()::atL1);
    public static final Trigger atL2 = new Trigger(RobotContainer.getElevator()::atL2);
    public static final Trigger atL3 = new Trigger(RobotContainer.getElevator()::atL3);
    public static final Trigger atL4 = new Trigger(RobotContainer.getElevator()::atL4);
    public static final Trigger atReef = atL2.or(atL3).or(atL4);
    public static final Trigger atNet = new Trigger(RobotContainer.getElevator()::atNet);
    public static final Trigger atProcessor = new Trigger(RobotContainer.getElevator()::atProcessor);
    public static final Trigger atIntake = new Trigger(RobotContainer.getElevator()::atIntake);
    public static final Trigger atHome = new Trigger(RobotContainer.getElevator()::atHome);
    public static final Trigger hasBall = new Trigger(RobotContainer.getAlgaeScore()::hasBall);
    public static final Trigger hasCoral = new Trigger(RobotContainer.getCoralScore()::hasCoral);
    public static final Trigger hasCage = new Trigger(RobotContainer.getCageGrabber()::hasCage);
    public static final Trigger atPosition = RobotContainer.getFunnelTilt()
            .isAtPosition()
            .and(RobotContainer.getWrist().isAtPosition())
            .and(RobotContainer.getElevator().isAtPosition());

    // Actions
    public static final Trigger actionScore = driver.rightBumper.and(teleop).or(auto_score);
    public static final Trigger actionIntake = driver.leftBumper.and(teleop).or(auto_intake);
    public static final Trigger actionUnjam = driver.start.and(teleop);
    public static final Trigger actionL1 = driver.A.and(teleop);
    public static final Trigger actionL2 = driver.B.and(teleop).or(auto_l2);
    public static final Trigger actionL3 = driver.X.and(teleop).or(auto_l3);
    public static final Trigger actionL4 = driver.Y.and(teleop).or(auto_l4);
    public static final Trigger actionReef = actionL2.or(actionL3).or(actionL4);
    public static final Trigger actionNet = driver.extraLeft.and(teleop);
    public static final Trigger actionProcessor = driver.extraRight.and(teleop);
    public static final Trigger actionLeftAim = driver.leftTrigger.and(teleop);
    public static final Trigger actionRightAim = driver.rightTrigger.and(teleop);
    public static final Trigger actionHome = new Trigger(() -> false);
    public static final Trigger actionClimbPrep = driver.upDpad.and(teleop);
    public static final Trigger actionClimb = driver.downDpad.and(teleop);
    public static final Trigger rezeroFieldHeading = driver.select.and(teleop);

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
