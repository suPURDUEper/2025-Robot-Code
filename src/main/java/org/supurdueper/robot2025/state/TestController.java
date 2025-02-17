package org.supurdueper.robot2025.state;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.supurdueper.lib.gamepad.Gamepad;
import org.supurdueper.lib.utils.ExpCurve;
import org.supurdueper.lib.utils.Util;

public class TestController extends Gamepad {

    public Trigger manualElevator;

    public TestController() {
        super(2, new ExpCurve(), 0.12, new ExpCurve(), 0.12, new ExpCurve(), 0.12);
        manualElevator = Util.teleop.and(rightStickX);
    }

    public double getManualElevator() {
        return rightStickCurve.calculate(getRightY());
    }
}
