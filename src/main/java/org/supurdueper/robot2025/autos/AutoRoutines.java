package org.supurdueper.robot2025.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import org.supurdueper.robot2025.Constants.FunnelTiltConstants;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.RobotStates;
import org.supurdueper.robot2025.subsystems.Elevator;
import org.supurdueper.robot2025.subsystems.Elevator.ElevatorHeight;
import org.supurdueper.robot2025.subsystems.Wrist;
import org.supurdueper.robot2025.subsystems.drive.Drivetrain;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final Drivetrain drivetrain;

    // center auto paths

    private final String centerAuto = "center_barge";

    private final String backLeftLeftToHp = "bl_left_to_hp";
    private final String backLeftRightToHp = "bl_right_to_hp";
    private final String backRightLeftToHp = "br_left_to_hp";
    private final String backRightRightToHp = "br_right_to_hp";
    private final String frontLeftToHp = "f_left_to_hp";
    private final String frontRightToHp = "f_right_to_hp";
    private final String frontLeftLeftToHp = "fl_left_to_hp";
    private final String frontLeftRightToHp = "fl_right_to_hp";
    private final String frontRightLeftToHp = "fr_left_to_hp";
    private final String frontRightRightToHp = "fr_right_to_hp";
    private final String leftHpToFront = "hp_left_to_f";
    private final String rightHpToFront = "hp_right_to_f";
    private final String hpToFrontLeft = "hp_to_fl";
    private final String hpToFrontRight = "hp_to_fr";
    private final String leftStart = "left_start";
    private final String centerStart = "center_start";
    private final String rightStart = "right_start";
    private final String hpToBackLeft = "hp_to_bl";
    private final String hptoBackRight = "hp_to_br";
    private final String centerToFirst = "center_to_first";
    private final String firstToBarge = "first_to_barge";
    private final String bargeToLeft = "barge_to_left";
    private final String secondToBarge = "left_to_barge";
    private final String bargeToRight = "barge_to_right";
    private final String thirdBarge = "right_to_barge";
    private final String bargeOffLine = "barge_off_line";

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        drivetrain = RobotContainer.getDrivetrain();
    }

    public Command intake() {
        return Commands.runOnce(() -> RobotStates.setAutointake(true))
                .andThen(Commands.runOnce(() -> RobotStates.setAutointake(false)));
    }

    public AutoRoutine threeCoralLeftRoutine() {

        AutoRoutine routine = m_factory.newRoutine("Three Coral Left");

        AutoTrajectory startToFirstCoral = routine.trajectory(leftStart);
        AutoTrajectory firstCoralToHp = routine.trajectory(backLeftLeftToHp);
        AutoTrajectory hpToSecondCoral = routine.trajectory(hpToFrontLeft);
        AutoTrajectory secondCoralToHp = routine.trajectory(frontLeftRightToHp);
        AutoTrajectory hpToThirdCoral = routine.trajectory(hpToFrontLeft);
        AutoTrajectory thirdCoralToHp = routine.trajectory(frontLeftLeftToHp);
        AutoTrajectory hpToFourthCoral = routine.trajectory(hpToBackLeft);

        routine.active()
                .onTrue(Commands.sequence(
                        m_factory.resetOdometry(leftStart), new ScheduleCommand(untangle()), startToFirstCoral.cmd()));

        hpToSecondCoral
                .active()
                .and(RobotContainer.getCoralScore()::hasCoral)
                .onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));
        hpToThirdCoral
                .active()
                .and(RobotContainer.getCoralScore()::hasCoral)
                .onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));
        hpToFourthCoral
                .active()
                .and(RobotContainer.getCoralScore()::hasCoral)
                .onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));

        startToFirstCoral
                .atTime(0.35)
                .onTrue(Commands.sequence(
                        new ScheduleCommand(
                                RobotContainer.getFunnelTilt().goToPosition(() -> FunnelTiltConstants.kIntakePosition)),
                        aimLeft(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        firstCoralToHp.cmd().asProxy()));
        firstCoralToHp.atTime(1).onTrue(intake());
        firstCoralToHp.chain(hpToSecondCoral);
        hpToSecondCoral
                .atTimeBeforeEnd(0.5)
                .onTrue(Commands.sequence(
                        aimRight(),
                        Commands.waitSeconds(0.1),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4())
                                .withTimeout(0.5),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimRight(false)),
                        secondCoralToHp.cmd().asProxy()));
        secondCoralToHp.atTime(1).onTrue(intake());
        secondCoralToHp.chain(hpToThirdCoral);
        hpToThirdCoral
                .atTimeBeforeEnd(0.5)
                .onTrue(Commands.sequence(
                        aimLeft(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4())
                                .withTimeout(0.5),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        thirdCoralToHp.cmd().asProxy()));
        thirdCoralToHp.atTime(1).onTrue(intake());
        thirdCoralToHp.chain(hpToFourthCoral);
        hpToFourthCoral
                .atTimeBeforeEnd(0.5)
                .onTrue(Commands.sequence(
                        aimRight(),
                        Commands.waitSeconds(0.15),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4())
                                .withTimeout(0.5),
                        score()));

        return routine;
    }

    public AutoRoutine oneCoralRightRoutine() {
        AutoRoutine routine = m_factory.newRoutine("Three Coral Right");

        AutoTrajectory startToFirstCoral = routine.trajectory(rightStart);
        AutoTrajectory firstCoralToHp = routine.trajectory(backRightRightToHp);
        AutoTrajectory hpToSecondCoral = routine.trajectory(hpToFrontRight);
        AutoTrajectory secondCoralToHp = routine.trajectory(frontRightLeftToHp);
        AutoTrajectory hpToThirdCoral = routine.trajectory(hpToFrontRight);
        AutoTrajectory thirdCoralToHp = routine.trajectory(frontRightRightToHp);
        AutoTrajectory hpToFourthCoral = routine.trajectory(hptoBackRight);

        routine.active()
                .onTrue(Commands.sequence(
                        m_factory.resetOdometry(rightStart), new ScheduleCommand(untangle()), startToFirstCoral.cmd()));

        hpToSecondCoral
                .active()
                .and(RobotContainer.getCoralScore()::hasCoral)
                .onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));
        hpToThirdCoral
                .active()
                .and(RobotContainer.getCoralScore()::hasCoral)
                .onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));
        hpToFourthCoral
                .active()
                .and(RobotContainer.getCoralScore()::hasCoral)
                .onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));

        startToFirstCoral
                .atTime(0.35)
                .onTrue(Commands.sequence(
                        new ScheduleCommand(
                                RobotContainer.getFunnelTilt().goToPosition(() -> FunnelTiltConstants.kIntakePosition)),
                        aimRight(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimRight(false)),
                        firstCoralToHp.cmd().asProxy()));
        firstCoralToHp.atTime(1).onTrue(intake());
        firstCoralToHp.chain(hpToSecondCoral);
        hpToSecondCoral
                .atTimeBeforeEnd(0.5)
                .onTrue(Commands.sequence(
                        aimLeft(),
                        Commands.waitSeconds(0.1),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4())
                                .withTimeout(0.5),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        secondCoralToHp.cmd().asProxy()));
        secondCoralToHp.atTime(1).onTrue(intake());
        secondCoralToHp.chain(hpToThirdCoral);
        hpToThirdCoral
                .atTimeBeforeEnd(0.5)
                .onTrue(Commands.sequence(
                        aimRight(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4())
                                .withTimeout(0.5),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimRight(false)),
                        thirdCoralToHp.cmd().asProxy()));
        thirdCoralToHp.atTime(1).onTrue(intake());
        thirdCoralToHp.chain(hpToFourthCoral);
        hpToFourthCoral
                .atTimeBeforeEnd(0.5)
                .onTrue(Commands.sequence(
                        aimLeft(),
                        Commands.waitSeconds(0.15),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4())
                                .withTimeout(0.5),
                        score()));

        return routine;

        // routine.active()
        //         .onTrue(Commands.sequence(
        //                 m_factory.resetOdometry(rightStart), new ScheduleCommand(untangle()),
        // startToFirstCoral.cmd()));

        // startToFirstCoral.active().onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));

        // startToFirstCoral
        //         .recentlyDone()
        //         .onTrue(Commands.sequence(
        //                 aimLeft(),
        //                 Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
        //                 Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
        //                 score(),
        //                 firstCoralToHp.cmd().asProxy()));
        // firstCoralToHp.atTime(1).onTrue(intake());
        // firstCoralToHp.chain(hpToSecondCoral);
        // hpToSecondCoral
        //         .recentlyDone()
        //         .onTrue(Commands.sequence(
        //                 aimRight(),
        //                 Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
        //                 Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
        //                 score(),
        //                 secondCoralToHp.cmd().asProxy()));
        // secondCoralToHp.atTime(1).onTrue(intake());
        // secondCoralToHp.chain(hpToThirdCoral);
        // hpToThirdCoral
        //         .recentlyDone()
        //         .onTrue(Commands.sequence(
        //                 aimLeft(),
        //                 Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
        //                 Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
        //                 score(),
        //                 thirdCoralToHp.cmd().asProxy()));
        // thirdCoralToHp.atTime(1).onTrue(intake());
        // thirdCoralToHp.chain(hpToFourthCoral);
        // hpToFourthCoral
        //         .recentlyDone()
        //         .onTrue(Commands.sequence(
        //                 aimRight(),
        //                 Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
        //                 Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
        //                 score()));

        // return routine;
    }

    public AutoRoutine centerAutoRightFirst() {
        AutoRoutine routine = m_factory.newRoutine("Center Auto Right First");

        AutoTrajectory startToFirst = routine.trajectory(centerAuto, 0);
        AutoTrajectory firstToBarge = routine.trajectory(centerAuto, 1);
        AutoTrajectory bargeToSecond = routine.trajectory(centerAuto, 2);
        AutoTrajectory secondToBarge = routine.trajectory(centerAuto, 3);
        AutoTrajectory bargeToThird = routine.trajectory(centerAuto, 4);
        AutoTrajectory thirdToBarge = routine.trajectory(centerAuto, 5);
        AutoTrajectory bargeBack = routine.trajectory(centerAuto, 6);

        Elevator elevator = RobotContainer.getElevator();
        Wrist wrist = RobotContainer.getWrist();

        routine.active()
                .onTrue(Commands.sequence(
                        m_factory.resetOdometry(centerStart), new ScheduleCommand(untangle()), startToFirst.cmd()));

        startToFirst
                .atTime(0)
                .onTrue(Commands.sequence(
                        aimLeftl2(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        firstToBarge.cmd().asProxy()));

        firstToBarge
                .atTime(0.6)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        firstToBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        algeaScore(),
                        bargeToSecond.cmd().asProxy()));

        bargeToSecond
                .atTimeBeforeEnd(0.8)
                .onTrue(Commands.sequence(
                        aimLeftl3(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        secondToBarge.cmd().asProxy()));

        secondToBarge
                .atTimeBeforeEnd(0.4)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        secondToBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        algeaScore(),
                        bargeToThird.cmd().asProxy()));

        bargeToThird
                .atTimeBeforeEnd(0.6)
                .onTrue(Commands.sequence(
                        aimLeftl3(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        thirdToBarge.cmd().asProxy()));

        thirdToBarge
                .atTimeBeforeEnd(0.25)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        thirdToBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        algeaScore(),
                        bargeBack.cmd().asProxy()));

        return routine;
    }

    public AutoRoutine centerAutoLeftFirst() {

        AutoRoutine routine = m_factory.newRoutine("Center Auto Left First");

        AutoTrajectory startToFirst = routine.trajectory(centerToFirst);
        AutoTrajectory firstBarge = routine.trajectory(firstToBarge);
        AutoTrajectory bargeToSecond = routine.trajectory(bargeToLeft);
        AutoTrajectory secondBarge = routine.trajectory(secondToBarge);
        AutoTrajectory bargeToThird = routine.trajectory(bargeToRight);
        AutoTrajectory thirdToBarge = routine.trajectory(thirdBarge);
        AutoTrajectory bargeBack = routine.trajectory(bargeOffLine);

        startToFirst
                .atTime(0)
                .onTrue(Commands.sequence(
                        aimLeftl2(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        firstBarge.cmd().asProxy()));

        firstBarge
                .atTime(0.6)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        firstBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        algeaScore(),
                        bargeToSecond.cmd().asProxy()));

        bargeToSecond
                .atTimeBeforeEnd(0.8)
                .onTrue(Commands.sequence(
                        aimLeftl3(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        secondBarge.cmd().asProxy()));

        secondBarge
                .atTimeBeforeEnd(0.4)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        secondBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        algeaScore(),
                        bargeToThird.cmd().asProxy()));

        bargeToThird
                .atTimeBeforeEnd(0.6)
                .onTrue(Commands.sequence(
                        aimLeftl3(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        thirdToBarge.cmd().asProxy()));

        thirdToBarge
                .atTimeBeforeEnd(0.25)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        thirdToBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        algeaScore(),
                        bargeBack.cmd().asProxy()));

        routine.active()
                .onTrue(Commands.sequence(
                        m_factory.resetOdometry(centerStart), new ScheduleCommand(untangle()), startToFirst.cmd()));

        return routine;
    }

    public Command aimLeft() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutol4(true)),
                Commands.runOnce(() -> RobotStates.setAutol4(false)),
                Commands.runOnce(() -> RobotStates.setAutoAimLeft(true)),
                Commands.waitUntil(RobotStates::isAimed));
    }

    public Command aimLeftl3() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutol3(true)),
                Commands.runOnce(() -> RobotStates.setAutol3(false)),
                Commands.runOnce(() -> RobotStates.setAutoAimLeft(true)),
                Commands.waitUntil(RobotStates::isAimed));
    }

    public Command aimLeftl2() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutol2(true)),
                Commands.runOnce(() -> RobotStates.setAutol2(false)),
                Commands.runOnce(() -> RobotStates.setAutoAimLeft(true)),
                Commands.waitUntil(RobotStates::isAimed));
    }

    public Command aimRight() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutol4(true)),
                Commands.runOnce(() -> RobotStates.setAutol4(false)),
                Commands.runOnce(() -> RobotStates.setAutoAimRight(true)),
                Commands.waitUntil(RobotStates::isAimed));
    }

    public Command score() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutoscore(true)),
                Commands.waitUntil(() -> RobotContainer.getCoralScore().scoredCoral())
                        .withTimeout(0.25),
                Commands.runOnce(() -> RobotStates.setAutoscore(false)));
    }

    public Command algeaScore() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutoscore(true)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> RobotStates.setAutoscore(false)));
    }

    public static Command untangle() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotContainer.getClimber().zero()),
                RobotContainer.getClimber().clearFunnel());
    }

    public static Command resetPose() {
        return Commands.runOnce(() -> RobotContainer.getDrivetrain()
                .resetPose(new Pose2d(
                        FieldConstants.fieldLength - Units.inchesToMeters(33.0 / 2.0),
                        FieldConstants.fieldWidth / 2,
                        Rotation2d.k180deg)));
    }

    public void chainWithDelay(AutoTrajectory a, AutoTrajectory b, double delaySeconds) {
        a.doneDelayed(delaySeconds).onTrue(b.cmd());
    }

    public Command nothing(String trajName) {
        return Commands.sequence(m_factory.resetOdometry(trajName), new ScheduleCommand(untangle()));
    }

    public Command nothingRight() {
        return nothing(rightStart);
    }

    public AutoRoutine nothingRightRoutine() {
        AutoRoutine routine = m_factory.newRoutine("Nothing Right");
        routine.active().onTrue(nothingRight());
        return routine;
    }

    public Command nothingLeft() {
        return nothing(leftStart);
    }
}
