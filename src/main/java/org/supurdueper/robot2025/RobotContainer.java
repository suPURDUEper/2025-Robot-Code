// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Getter;
import org.supurdueper.robot2025.autos.AutoRoutines;
import org.supurdueper.robot2025.state.Driver;
import org.supurdueper.robot2025.subsystems.AlgaeScore;
import org.supurdueper.robot2025.subsystems.CageGrabber;
import org.supurdueper.robot2025.subsystems.Climber;
import org.supurdueper.robot2025.subsystems.CoralScore;
import org.supurdueper.robot2025.subsystems.Elevator;
import org.supurdueper.robot2025.subsystems.Funnel;
import org.supurdueper.robot2025.subsystems.FunnelTilt;
import org.supurdueper.robot2025.subsystems.drive.Drivetrain;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;
import org.supurdueper.robot2025.IntakeCommand;

public class RobotContainer {

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
        algaeScore = new AlgaeScore();
        cageGrabber = new CageGrabber();
        climber = new Climber();
        coralScore = new CoralScore();
        // elevator = new Elevator();
        funnel = new Funnel();
        funnelTilt = new FunnelTilt();
        driver = new Driver();
        drivetrain = TunerConstants.createDrivetrain();
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        //One Driver Controls
            //The along with will be putting the wrist at the propper angle
        joystick.a().onTrue((elevator.l1()).alongWith(null));
        joystick.b().onTrue((elevator.l2()).alongWith());
        joystick.x().onTrue((elevator.l3()).alongWith());
        joystick.y().onTrue((elevator.l4()).alongWith());

        joystick.rightTrigger().onTrue(coralScore.ScoreCoral());
        // Fix this later, joystick.rightBumper().onTrue(IntakeCommand());
        joystick.leftTrigger().onTrue(algaeScore.scoreNet());
        joystick.leftBumper().onTrue(algaeScore.scoreProcessor());

        //null is net angle
        joystick.povUp().onTrue((elevator.net()).alongWith(null));
        //null is processor angle
        joystick.povDown().onTrue((elevator.processor()).alongWith(null));

    





        //Two driver Controls
        /* 

        // Operator
        joystick2.a().onTrue((elevator.l1()).alongWith(null));
        joystick2.b().onTrue((elevator.l2()).alongWith());
        joystick2.x().onTrue((elevator.l3()).alongWith());
        joystick2.y().onTrue((elevator.l4()).alongWith());

        //Driver


        */



    
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
