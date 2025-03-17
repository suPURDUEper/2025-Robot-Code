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
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.RobotStates;
import org.supurdueper.robot2025.subsystems.drive.Drivetrain;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final Drivetrain drivetrain;

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
    private final String rightStart = "right_start";

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        drivetrain = RobotContainer.getDrivetrain();
    }

    public Command intake() {
        return Commands.runEnd(() -> RobotStates.setAutointake(true), () -> RobotStates.setAutointake(false));
    }

    public AutoRoutine oneCoralLeftRoutine() {

        AutoRoutine routine = m_factory.newRoutine("One Coral Left");

        AutoTrajectory startToFirstCoral = routine.trajectory(leftStart);
        AutoTrajectory firstCoralToHp = routine.trajectory(backLeftLeftToHp);
        AutoTrajectory hpToSecondCoral = routine.trajectory(hpToFrontLeft);
        AutoTrajectory secondCoralToHp = routine.trajectory(frontLeftLeftToHp);
        AutoTrajectory hpToThirdCoral = routine.trajectory(hpToFrontLeft);

        routine.active()
                .onTrue(Commands.sequence(
                        m_factory.resetOdometry(leftStart), new ScheduleCommand(untangle()), startToFirstCoral.cmd()));

        startToFirstCoral
                .recentlyDone()
                .onTrue(Commands.sequence(
                        aimLeft(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
                        score(),
                        firstCoralToHp.cmd().asProxy()));

        firstCoralToHp.chain(hpToSecondCoral);

        return routine;
    }

    public Command aimLeft() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutol4(true)),
                Commands.runOnce(() -> RobotStates.setAutol4(false)),
                Commands.runOnce(() -> RobotStates.setAutoAimLeft(true)),
                Commands.waitUntil(RobotStates::isAimed),
                Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)));
    }

    public Command aimRight() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutol4(true)),
                Commands.runOnce(() -> RobotStates.setAutol4(false)),
                Commands.runOnce(() -> RobotStates.setAutoAimRight(true)),
                Commands.waitUntil(RobotStates::isAimed),
                Commands.runOnce(() -> RobotStates.setAutoAimRight(false)));
    }

    public Command score() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutoscore(true)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> RobotStates.setAutoscore(false)));
    }

    public static Command untangle() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotContainer.getClimber().zero()),
                // Commands.runOnce(() -> RobotContainer.getElevator().zero()),
                RobotContainer.getClimber().clearFunnel().withTimeout(1.5),
                RobotContainer.getFunnelTilt().intake().withTimeout(0.75),
                RobotContainer.getClimber().home());
    }

    public static Command resetPose() {
        return Commands.runOnce(() -> RobotContainer.getDrivetrain()
                .resetPose(new Pose2d(
                        FieldConstants.fieldLength - Units.inchesToMeters(33.0 / 2.0),
                        FieldConstants.fieldWidth / 2,
                        Rotation2d.k180deg)));
    }

    public void chain(AutoTrajectory a, AutoTrajectory b, double delaySeconds) {
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
