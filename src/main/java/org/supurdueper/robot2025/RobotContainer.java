// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025;

import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import org.supurdueper.robot2025.state.Driver;
import org.supurdueper.robot2025.state.TestController;
import org.supurdueper.robot2025.subsystems.AlgaeScore;
import org.supurdueper.robot2025.subsystems.CageGrabber;
import org.supurdueper.robot2025.subsystems.Climber;
import org.supurdueper.robot2025.subsystems.CoralScore;
import org.supurdueper.robot2025.subsystems.Elevator;
import org.supurdueper.robot2025.subsystems.Funnel;
import org.supurdueper.robot2025.subsystems.FunnelTilt;
import org.supurdueper.robot2025.subsystems.Lights;
import org.supurdueper.robot2025.subsystems.Vision;
import org.supurdueper.robot2025.subsystems.Wrist;
import org.supurdueper.robot2025.subsystems.drive.Drivetrain;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class RobotContainer {

    @Getter
    private static Drivetrain drivetrain;

    @Getter
    private static AlgaeScore algaeScore;

    @Getter
    private static CageGrabber cageGrabber;

    @Getter
    private static Climber climber;

    @Getter
    private static CoralScore coralScore;

    @Getter
    private static Elevator elevator;

    @Getter
    private static Wrist wrist;

    @Getter
    private static Funnel funnel;

    @Getter
    private static FunnelTilt funnelTilt;

    @Getter
    private static Vision vision;

    @Getter
    private static Driver driver;

    @Getter
    private static Lights lights;

    @Getter
    private static TestController testController;

    public RobotContainer() {
        algaeScore = new AlgaeScore();
        cageGrabber = new CageGrabber();
        climber = new Climber();
        coralScore = new CoralScore();
        elevator = new Elevator();
        wrist = new Wrist();
        funnel = new Funnel();
        funnelTilt = new FunnelTilt();
        driver = new Driver();
        testController = new TestController();
        drivetrain = TunerConstants.createDrivetrain();
        vision = new Vision();
        lights = new Lights();

        configureBindings();
    }

    public void configureBindings() {
        climber.setDefaultCommand(climber.setVoltage(testController::getManualElevatorVoltage));
        testController.X.onTrue(climber.home());
        testController.Y.onTrue(climber.climbPrep());
        testController.A.onTrue(climber.retract());
        testController.select.onTrue(Commands.runOnce(() -> climber.zero()));
        testController.downDpad.onTrue(climber.disengageRatchet());
        testController.upDpad.onTrue(climber.engageRatchet());
        funnelTilt.setDefaultCommand(funnelTilt.setVoltage(testController::getManualWristVoltage));

        // Pose2d aGoal = new Pose2d(0, 0, Rotation2d.kZero);
        // Pose2d xGoal = new Pose2d(1, 1, Rotation2d.kZero);
        // Pose2d bGoal = new Pose2d(1, -1, Rotation2d.kZero);
        // Pose2d yGoal = new Pose2d(2, 0, Rotation2d.kZero);

        // testController.A.whileTrue(RobotContainer.getDrivetrain().driveStates.driveToPose(() -> aGoal));
        // testController.B.whileTrue(RobotContainer.getDrivetrain().driveStates.driveToPose(() -> bGoal));
        // testController.X.whileTrue(RobotContainer.getDrivetrain().driveStates.driveToPose(() -> xGoal));
        // testController.Y.whileTrue(RobotContainer.getDrivetrain().driveStates.driveToPose(() -> yGoal));
    }
}
