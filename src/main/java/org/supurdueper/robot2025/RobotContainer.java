// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.robot2025;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.Getter;
import org.supurdueper.robot2025.autos.AutoRoutines;
import org.supurdueper.robot2025.state.Driver;
import org.supurdueper.robot2025.subsystems.drive.Drivetrain;
import org.supurdueper.robot2025.subsystems.drive.generated.TunerConstants;

public class RobotContainer {

    @Getter
    private static Drivetrain drivetrain;

    // @Getter
    // private static AlgaeScore algaeScore;

    // @Getter
    // private static CageGrabber cageGrabber;

    // @Getter
    // private static Climber climber;

    // @Getter
    // private static CoralScore coralScore;

    // @Getter
    // private static Elevator elevator;

    // @Getter
    // private static Funnel funnel;

    // @Getter
    // private static FunnelTilt funnelTilt;

    @Getter
    private static Driver driver;

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        // algaeScore = new AlgaeScore();
        // cageGrabber = new CageGrabber();
        // climber = new Climber();
        // coralScore = new CoralScore();
        // // elevator = new Elevator();
        // funnel = new Funnel();
        // funnelTilt = new FunnelTilt();
        driver = new Driver();
        drivetrain = TunerConstants.createDrivetrain();
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
}
