package org.supurdueper.robot2025.state;

import org.supurdueper.lib.gamepad.Gamepad;
import org.supurdueper.lib.utils.ExpCurve;

public class TestController extends Gamepad {

    private static double kDeadzone = 0.15;

    public TestController() {
        super(
                3,
                new ExpCurve(1, 0, 1, kDeadzone),
                0.15,
                new ExpCurve(1, 0, 1, kDeadzone),
                0.15,
                new ExpCurve(1, 0, 0.5, kDeadzone),
                kDeadzone);
    }

    public double getManualElevatorVoltage() {
        return leftStickCurve.calculate(getLeftY()) * 12;
    }

    public double getManualWristVoltage() {
        return rightStickCurve.calculate(getRightY()) * 12;
    }
}
