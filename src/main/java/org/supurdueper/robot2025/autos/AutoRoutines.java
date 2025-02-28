package org.supurdueper.robot2025.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
    private final String rightTrajName = "auto3coralright";
    private final String leftTrajName = "auto3coralleft";

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        drivetrain = RobotContainer.getDrivetrain();
    }

    public Command l4() {
        return Commands.runEnd(() -> RobotStates.setAutol4(true), () -> RobotStates.setAutol4(false));
    }

    public Command l3() {
        return Commands.runEnd(() -> RobotStates.setAutol3(true), () -> RobotStates.setAutol3(false));
    }

    public Command l2() {
        return Commands.runEnd(() -> RobotStates.setAutol2(true), () -> RobotStates.setAutol2(false));
    }

    public Command intake() {
        return Commands.runEnd(() -> RobotStates.setAutointake(true), () -> RobotStates.setAutointake(false));
    }

    public Command score() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutoscore(true)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> RobotStates.setAutoscore(false)));
    }

    public Command dumpCoral() {
        return RobotContainer.getAlgaeScore().scoreProcessor();
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

    public Command threeCoralAutoRight() {
        return threeCoralAuto(rightTrajName);
    }

    public Command threeCoralAutoLeft() {
        return threeCoralAuto(leftTrajName);
    }

    public Command threeCoralClearAlgaeAutoRight() {
        return threeCoralClearBall(rightTrajName);
    }

    public Command threeCoralClearAlgaeAutoLeft() {
        return threeCoralClearBall(leftTrajName);
    }

    public AutoRoutine threeCoralAutoRoutine(String trajName) {
        AutoRoutine routine = m_factory.newRoutine(trajName);

        AutoTrajectory startToFirstCoral = routine.trajectory(trajName, 0);
        AutoTrajectory firstCoralToHp = routine.trajectory(trajName, 1);
        AutoTrajectory hpToSecondCoral = routine.trajectory(trajName, 2);
        AutoTrajectory secondCoralToHp = routine.trajectory(trajName, 3);
        AutoTrajectory hpToThirdCoral = routine.trajectory(trajName, 4);

        routine.active().onTrue(Commands.sequence(
                m_factory.resetOdometry(trajName),
                new ScheduleCommand(untangle()),
                startToFirstCoral.cmd()));

        // Stop the drive at the end of every routine
        routine.anyDone(startToFirstCoral, firstCoralToHp, hpToSecondCoral, secondCoralToHp, hpToThirdCoral)
                .onTrue(drivetrain.stop());

        chainWithScore(startToFirstCoral, firstCoralToHp);
        chainWithHpLoad(firstCoralToHp, hpToSecondCoral);
        chainWithScore(hpToSecondCoral, secondCoralToHp);
        chainWithHpLoad(secondCoralToHp, hpToThirdCoral);

        return routine;
    }

    public Command threeCoralAuto(String trajName) {

        Command startToFirstCoral = m_factory.trajectoryCmd(trajName, 0);
        Command firstCoralToHp = m_factory.trajectoryCmd(trajName, 1);
        Command hpToSecondCoral = m_factory.trajectoryCmd(trajName, 2);
        Command secondCoralToHp = m_factory.trajectoryCmd(trajName, 3);
        Command hpToThirdCoral = m_factory.trajectoryCmd(trajName, 4);

        return Commands.sequence(
                m_factory.resetOdometry(trajName),
                new ScheduleCommand(untangle()),
                Commands.deadline(startToFirstCoral, l4()),
                drivetrain.stop(),
                score(),
                Commands.deadline(firstCoralToHp, intake()),
                drivetrain.stop(),
                Commands.waitUntil(RobotStates.hasCoral),
                Commands.deadline(hpToSecondCoral, l4()),
                drivetrain.stop(),
                score(),
                Commands.waitSeconds(0.5),
                Commands.deadline(secondCoralToHp, intake()),
                drivetrain.stop(),
                Commands.waitUntil(RobotStates.hasCoral),
                Commands.deadline(hpToThirdCoral, l4()),
                drivetrain.stop(),
                score());
    }

    public Command threeCoralClearBall(String trajName) {

        Command startToFirstCoral = m_factory.trajectoryCmd(trajName, 0);
        Command firstCoralToHp = m_factory.trajectoryCmd(trajName, 1);
        Command hpToSecondCoral = m_factory.trajectoryCmd(trajName, 2);
        Command secondCoralToHp = m_factory.trajectoryCmd(trajName, 3);
        Command hpToThirdCoral = m_factory.trajectoryCmd(trajName, 4);
        Command thirdCoralBackwards = m_factory.trajectoryCmd(trajName, 5);

        return Commands.sequence(
                m_factory.resetOdometry(trajName),
                new ScheduleCommand(untangle()),
                Commands.deadline(startToFirstCoral, l3()),
                drivetrain.stop(),
                score(),
                Commands.deadline(
                        firstCoralToHp,
                        Commands.waitSeconds(0.75).andThen(dumpCoral()).andThen(intake())),
                drivetrain.stop(),
                Commands.waitUntil(RobotStates.hasCoral),
                Commands.deadline(hpToSecondCoral, l4()),
                drivetrain.stop(),
                score(),
                Commands.waitSeconds(0.5),
                Commands.deadline(secondCoralToHp, intake()),
                drivetrain.stop(),
                Commands.waitUntil(RobotStates.hasCoral),
                Commands.deadline(hpToThirdCoral, l2()),
                drivetrain.stop(),
                score(),
                thirdCoralBackwards);
    }

    public void chainWithScore(AutoTrajectory a, AutoTrajectory b) {
        a.atTimeBeforeEnd(1).onTrue(l4());
        a.done().onTrue(score().andThen(b.cmd()));
    }

    public void chainWithHpLoad(AutoTrajectory a, AutoTrajectory b) {
        a.atTimeBeforeEnd(0.5).onTrue(intake());
        a.done().and(RobotStates.hasCoral).onTrue(b.cmd());
    }

    public SwerveRequest stop() {
        return new SwerveRequest.Idle();
    }
}
