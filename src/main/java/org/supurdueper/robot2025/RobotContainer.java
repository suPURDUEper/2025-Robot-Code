// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025;

import static edu.wpi.first.units.Units.Degrees;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.Getter;
import org.supurdueper.robot2025.autos.AutoRoutines;
import org.supurdueper.robot2025.state.Driver;
import org.supurdueper.robot2025.state.TestController;
import org.supurdueper.robot2025.subsystems.AlgaeScore;
import org.supurdueper.robot2025.subsystems.CageGrabber;
import org.supurdueper.robot2025.subsystems.CoralScore;
import org.supurdueper.robot2025.subsystems.Elevator;
import org.supurdueper.robot2025.subsystems.Funnel;
import org.supurdueper.robot2025.subsystems.FunnelTilt;
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

    // @Getter
    // private static Climber climber;

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
    private static Driver driver;

    @Getter
    private static TestController testController;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        algaeScore = new AlgaeScore();
        cageGrabber = new CageGrabber();
        // climber = new Climber();
        coralScore = new CoralScore();
        elevator = new Elevator();
        wrist = new Wrist();
        funnel = new Funnel();
        funnelTilt = new FunnelTilt();
        driver = new Driver();
        testController = new TestController();
        drivetrain = TunerConstants.createDrivetrain();
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    public void configureBindings() {
        testController.leftStickY.whileTrue(elevator.setVoltage(testController::getManualElevatorVoltage));
        testController.A.whileTrue(wrist.goToPosition(Degrees.of(55)));
        testController.X.whileTrue(wrist.goToPosition(Degrees.of(80)));
        testController.Y.whileTrue(wrist.goToPosition(Degrees.of(105)));
        wrist.setDefaultCommand(wrist.setVoltage(testController::getManualWristVoltage));
        // testController.A.whileTrue(funnel.intake());
        // testController.B.whileTrue(coralScore.runForwards());
        // testController.X.whileTrue(algaeScore.intake());
        // testController.Y.whileTrue(algaeScore.scoreNet());
        testController.rightBumper.whileTrue(cageGrabber.runForwards());
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
