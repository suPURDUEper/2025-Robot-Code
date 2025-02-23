package org.supurdueper.robot2025.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.supurdueper.robot2025.FieldConstants;
import org.supurdueper.robot2025.RobotContainer;
import org.supurdueper.robot2025.state.RobotStates;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
        factory.bind("L4", l4()).bind("Intake", intake());
    }

    public Command l4() {
        return Commands.runEnd(() -> RobotStates.setAutol4(true), () -> RobotStates.setAutol4(false));
    }

    public Command intake() {
        return Commands.runEnd(() -> RobotStates.setAutointake(true), () -> RobotStates.setAutointake(false));
    }

    public Command score() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutoscore(true)),
                Commands.waitSeconds(0.75),
                Commands.runOnce(() -> RobotStates.setAutoscore(false)));
    }

    public static Command resetPose() {
        return Commands.runOnce(() -> RobotContainer.getDrivetrain()
                .resetPose(new Pose2d(
                        FieldConstants.fieldLength - Units.inchesToMeters(33.0 / 2.0),
                        FieldConstants.fieldWidth / 2,
                        Rotation2d.k180deg)));
    }

    public AutoRoutine threeCoralAuto() {
        String trajName = "Auto3Coral";
        final AutoRoutine routine = m_factory.newRoutine("Three Coral Auto");

        final AutoTrajectory startToFirstCoral = routine.trajectory(trajName, 0);
        final AutoTrajectory firstCoralToHp = routine.trajectory(trajName, 1);
        final AutoTrajectory hpToSecondCoral = routine.trajectory(trajName, 2);
        final AutoTrajectory secondCoralToHp = routine.trajectory(trajName, 3);
        final AutoTrajectory hpToThirdCoral = routine.trajectory(trajName, 4);

        // Score when we finish any of the trajectories going to the reef
        routine.anyDone(startToFirstCoral, hpToSecondCoral, hpToThirdCoral).onTrue(score());

        // Setup chain with delays
        chain(startToFirstCoral, firstCoralToHp, 1);
        chain(firstCoralToHp, hpToSecondCoral, 0.5);
        chain(hpToSecondCoral, secondCoralToHp, 1);
        chain(secondCoralToHp, hpToThirdCoral, 0.5);

        // Reset odometry and start the first trajectory
        startToFirstCoral.active().onTrue(startToFirstCoral.resetOdometry().andThen(startToFirstCoral.cmd()));

        return routine;
    }

    public void chain(AutoTrajectory a, AutoTrajectory b, double delaySeconds) {
        a.doneDelayed(delaySeconds).onTrue(b.cmd());
    }
}
