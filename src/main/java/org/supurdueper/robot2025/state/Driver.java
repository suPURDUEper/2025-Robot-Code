package org.supurdueper.robot2025.state;

import static org.supurdueper.robot2025.Constants.DriverConstants.*;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import org.supurdueper.lib.gamepad.Gamepad;
import org.supurdueper.lib.utils.Util;

public class Driver extends Gamepad {

    // // Triggers, these would be robot states such as ampReady, intake, visionAim,
    // etc.
    // // If triggers need any of the config values set them in the constructor
    // /* A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
    public final Trigger fn = leftTrigger;
    public final Trigger noFn = fn.not();

    // // TODO: Finalize Buttons
    public final Trigger intake_lb = leftBumper.and(noFn, teleop);
    public final Trigger score_rb = rightBumper.and(noFn, teleop);
    public final Trigger l1_A = A.and(noFn, teleop);
    public final Trigger l2_B = B.and(noFn, teleop);
    public final Trigger l3_X = X.and(noFn, teleop);
    public final Trigger l4_Y = Y.and(noFn, teleop);

    
    public final Trigger home_fA = A.and(fn, teleop);
    public final Trigger processor_fX = X.and(fn, teleop);
    public final Trigger net_fY = Y.and(fn, teleop);



    // // Drive Triggers
    // public final Trigger upReorient = upDpad.and(fn, teleop);
    // public final Trigger leftReorient = leftDpad.and(fn, teleop);
    // public final Trigger downReorient = downDpad.and(fn, teleop);
    // public final Trigger rightReorient = rightDpad.and(fn, teleop);

    // /* Use the right stick to set a cardinal direction to aim at */
    public final Trigger driving;
    public final Trigger steer;

    // public final Trigger snapSteer = Trigger.kFalse;

    // public final Trigger fpv_rs = rightStickClick.and(teleop); // Remapped to
    // Right back button

    // // DISABLED TRIGGERS
    // public final Trigger coastOn_dB = disabled.and(B);
    // public final Trigger coastOff_dA = disabled.and(A);

    // // TEST TRIGGERS
    // public final Trigger tuneElevator_tB = testMode.and(B);
    @Getter
    @Setter
    private boolean isSlowMode = false;

    @Getter
    @Setter
    private boolean isTurboMode = false;

    /** Create a new Pilot with the default name and port. */
    public Driver() {
        super(0, kLeftStickCurve, kDeadzone, kRightStickCurve, kDeadzone, kTriggerCurve, kDeadzone);
        driving = Util.teleop.and(leftStickX.or(leftStickY));
        steer = Util.teleop.and(rightStickX.or(rightStickY));
    }

    // DRIVE METHODS
    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        if (isSlowMode) {
            fwdPositive *= Math.abs(kSlowModeScalor);
        }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        if (isSlowMode) {
            leftPositive *= Math.abs(kSlowModeScalor);
        }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        if (isSlowMode) {
            ccwPositive *= Math.abs(kSlowModeScalor);
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(kTurboModeScalor);
        } else {
            ccwPositive *= Math.abs(kDefaultTurnScalor);
        }
        return -1 * ccwPositive; // invert the value
    }
}
