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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ironmaple.simulation.SimulatedArena;
import org.supurdueper.BuildConstants;
import org.supurdueper.lib.subsystems.SupurdueperRobot;
import org.supurdueper.robot2025.autos.AutoRoutines;
import org.supurdueper.robot2025.subsystems.Vision;

public class Robot extends SupurdueperRobot {

    private final RobotContainer m_robotContainer;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser choreoAutoChooser;
    // private final SendableChooser<Command> autoChooser;

    public Robot() {
        m_robotContainer = new RobotContainer();
        autoFactory = RobotContainer.getDrivetrain().createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);
        choreoAutoChooser = new AutoChooser();
        choreoAutoChooser.addCmd("Nothing Right", autoRoutines::nothingRight);
        choreoAutoChooser.addCmd("Nothing Left", autoRoutines::nothingLeft);
        choreoAutoChooser.addRoutine("Three Coral Left", autoRoutines::oneCoralLeftRoutine);
        choreoAutoChooser.addRoutine("Three Coral Right", autoRoutines::oneCoralRightRoutine);
        SmartDashboard.putData("Choreo Auto Chooser", choreoAutoChooser);
        // RobotModeTriggers.autonomous().whileTrue(choreoAutoChooser.selectedCommandScheduler());
    }

    @Override
    public void robotInit() {
        DogLog.setOptions(new DogLogOptions()
                .withLogExtras(false)
                .withCaptureDs(false)
                .withNtPublish(Constants.publishToNT)
                .withCaptureNt(false));
        // Record metadata
        DogLog.log("Git/ProjectName", BuildConstants.MAVEN_NAME);
        DogLog.log("Git/BuildDate", BuildConstants.BUILD_DATE);
        DogLog.log("Git/GitSHA", BuildConstants.GIT_SHA);
        DogLog.log("Git/GitDate", BuildConstants.GIT_DATE);
        DogLog.log("Git/GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                DogLog.log("Git/GitDirty", "All changes committed");
                break;
            case 1:
                DogLog.log("Git/GitDirty", "Uncomitted changes");
                break;
            default:
                DogLog.log("Git/GitDirty", "Unknown");
                break;
        }

        // Limelight port fowarding
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "10.74.57.11", port);
            PortForwarder.add(port + 100, "10.74.57.12", port);
        }
        FieldConstants.getAprilTagPose(7);
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 1);
        double startTime = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().run();
        double endTime = Timer.getFPGATimestamp();
        Threads.setCurrentThreadPriority(false, 0);
        DogLog.log("Loop Time", endTime - startTime);
    }

    @Override
    public void disabledInit() {
        resetCommandsAndButtons();
        Vision.setDisabled();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Vision.setEnabled();
        Vision.setAprilTagFilter();
        choreoAutoChooser.selectedCommandScheduler().schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        resetCommandsAndButtons();
        Vision.setEnabled();
        Vision.setAprilTagFilter();
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
