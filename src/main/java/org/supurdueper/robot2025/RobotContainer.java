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

        configureBindings();
    }

    public void configureBindings() {
        // climber.setDefaultCommand(climber.setVoltage(testController::getManualElevatorVoltage));
        elevator.setDefaultCommand(elevator.runCurrent(() -> testController.getManualElevatorVoltage() * 4));
        // testController.Y.onTrue(climber.home());
        // testController.X.onTrue(climber.climbPrep());
        // testController.rightStickY.onTrue((funnelTilt.setVoltage(testController::getManualWristVoltage)));
        testController.start.onTrue(Commands.runOnce(() -> climber.zero(), climber));
        // testController.A.onTrue(funnelTilt.startingPosition());
        funnelTilt.setDefaultCommand(funnelTilt.setVoltage(testController::getManualWristVoltage));
        testController.A.onTrue(elevator.setStateAndGoToHeight(Elevator.ElevatorHeight.Home));
        testController.X.onTrue(elevator.setStateAndGoToHeight(Elevator.ElevatorHeight.L3));
        testController.B.onTrue(elevator.setStateAndGoToHeight(Elevator.ElevatorHeight.L2));
        testController.Y.onTrue(elevator.setStateAndGoToHeight(Elevator.ElevatorHeight.L4));
        // testController.A.onTrue(climber.disengageRatchet());
    }
}
