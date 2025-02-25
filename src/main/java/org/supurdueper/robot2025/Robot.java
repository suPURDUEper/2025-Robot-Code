// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import org.ironmaple.simulation.SimulatedArena;
import org.supurdueper.BuildConstants;
import org.supurdueper.lib.subsystems.SupurdueperRobot;
import org.supurdueper.robot2025.autos.AutoRoutines;

public class Robot extends SupurdueperRobot {

    private final RobotContainer m_robotContainer;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final SendableChooser<Command> autoChooser;
    private final AutoChooser choreoAutoChooser;

    public Robot() {
        m_robotContainer = new RobotContainer();
        autoFactory = RobotContainer.getDrivetrain().createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Three Coral Right", autoRoutines.threeCoralAuto());
        autoChooser.addOption("Nothing", Commands.none());
        SmartDashboard.putData("Auto Chooser", autoChooser);

        choreoAutoChooser = new AutoChooser();
        // choreoAutoChooser.addCmd("Three Coral Right", autoRoutines::threeCoralAuto);
        // choreoAutoChooser.addCmd("Nothing", Commands::none);
        // SmartDashboard.putData("Auto Chooser", choreoAutoChooser);
        // RobotModeTriggers.autonomous().whileTrue(choreoAutoChooser.selectedCommandScheduler());
    }

    @Override
    public void robotInit() {
        DogLog.setOptions(new DogLogOptions()
                .withLogExtras(true)
                .withCaptureDs(true)
                .withNtPublish(true)
                .withCaptureNt(true));
        // Record metadata
        DogLog.log("ProjectName", BuildConstants.MAVEN_NAME);
        DogLog.log("BuildDate", BuildConstants.BUILD_DATE);
        DogLog.log("GitSHA", BuildConstants.GIT_SHA);
        DogLog.log("GitDate", BuildConstants.GIT_DATE);
        DogLog.log("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                DogLog.log("GitDirty", "All changes committed");
                break;
            case 1:
                DogLog.log("GitDirty", "Uncomitted changes");
                break;
            default:
                DogLog.log("GitDirty", "Unknown");
                break;
        }

        // Limelight port fowarding
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "10.74.57.11", port);
            PortForwarder.add(port + 100, "10.74.57.12", port);
        }
    }

    @Override
    public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run();
        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        resetCommandsAndButtons();
        autoChooser.getSelected().schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        resetCommandsAndButtons();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        resetCommandsAndButtons();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        DogLog.log("Simulation/CoralPoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        DogLog.log("Simulation/AlgaePoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    /**
     * This method cancels all commands and returns subsystems to their default commands and the gamepad configs are
     * reset so that new bindings can be assigned based on mode This method should be called when each mode is
     * initialized
     */
    public void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Bind Triggers for all subsystems
        bindCommands();
        m_robotContainer.configureBindings();
    }
}
