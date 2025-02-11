package org.supurdueper.lib.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;

public class SupurdueperRobot extends TimedRobot {

    /** Create a single static instance of all of your subsystems */
    private static final ArrayList<SupurdueperSubsystem> subsystems = new ArrayList<>();

    public static void add(SupurdueperSubsystem subsystem) {
        subsystems.add(subsystem);
    }

    public SupurdueperRobot() {
        super();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    // protected void setupDefaultCommands() {
    //     // Setup Default Commands for all subsystems
    //     subsystems.forEach(SupurdueperSubsystem::setupDefaultCommand);
    // }

    protected void bindCommands() {
        // Bind Triggers for all subsystems
        subsystems.forEach(SupurdueperSubsystem::bindCommands);
    }
}
