// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.supurdueper.robot2025.autos.AutoRoutines;
=======
import lombok.Getter;
import org.supurdueper.robot2025.autos.AutoRoutines;
import org.supurdueper.robot2025.state.Driver;
>>>>>>> 3a2b718afdde7751fdd6fb0f5dceee38b32c6d7c
import org.supurdueper.robot2025.subsystems.AlgaeScore;
import org.supurdueper.robot2025.subsystems.CageGrabber;
import org.supurdueper.robot2025.subsystems.Climber;
import org.supurdueper.robot2025.subsystems.CoralScore;
import org.supurdueper.robot2025.subsystems.Elevator;
import org.supurdueper.robot2025.subsystems.Funnel;
import org.supurdueper.robot2025.subsystems.FunnelTilt;
<<<<<<< HEAD
import org.supurdueper.robot2025.subsystems.Lights;
import org.supurdueper.robot2025.subsystems.Wrist;
import org.supurdueper.robot2025.subsystems.drive.Drive;
=======
import org.supurdueper.robot2025.subsystems.drive.Drivetrain;
>>>>>>> 3a2b718afdde7751fdd6fb0f5dceee38b32c6d7c
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class RobotContainer {
    final Climber climber;
    final Elevator elevator;
    final Wrist wrist;
    final Lights lights;
    final Funnel funnel;
    final FunnelTilt funneltilt;
    final AlgaeScore algaescore;
    final CoralScore coralscore;
    final CageGrabber cagegrabber;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

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
    private static Funnel funnel;

    @Getter
    private static FunnelTilt funnelTilt;

    @Getter
    private static Driver driver;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
<<<<<<< HEAD

        cagegrabber = new CageGrabber();
        coralscore = new CoralScore();
        algaescore = new AlgaeScore();
        climber = new Climber();
        funnel = new Funnel();
        funneltilt = new FunnelTilt();
        wrist = new Wrist();
        elevator = new Elevator();
        lights = new Lights();

=======
        algaeScore = new AlgaeScore();
        cageGrabber = new CageGrabber();
        climber = new Climber();
        coralScore = new CoralScore();
        elevator = new Elevator();
        funnel = new Funnel();
        funnelTilt = new FunnelTilt();
        driver = new Driver();
        drivetrain = TunerConstants.createDrivetrain();
>>>>>>> 3a2b718afdde7751fdd6fb0f5dceee38b32c6d7c
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        joystick.leftBumper().whileTrue(funnel.intake().alongWith(coralscore.loadCoral()));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
