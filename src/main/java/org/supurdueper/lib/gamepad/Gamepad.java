package org.supurdueper.lib.gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.supurdueper.lib.Telemetry;
import org.supurdueper.lib.utils.ExpCurve;

// Gamepad class
public abstract class Gamepad extends SubsystemBase {
    private Alert disconnectedAlert;

    public static final Trigger kFalse = new Trigger(() -> false);

    private CommandXboxController xboxController;
    public Trigger A = kFalse;
    public Trigger B = kFalse;
    public Trigger X = kFalse;
    public Trigger Y = kFalse;
    public Trigger leftBumper = kFalse;
    public Trigger rightBumper = kFalse;
    public Trigger leftTrigger = kFalse;
    public Trigger rightTrigger = kFalse;
    public Trigger leftStickClick = kFalse;
    public Trigger rightStickClick = kFalse;
    public Trigger start = kFalse;
    public Trigger select = kFalse;
    public Trigger upDpad = kFalse;
    public Trigger downDpad = kFalse;
    public Trigger leftDpad = kFalse;
    public Trigger rightDpad = kFalse;
    public Trigger leftStickY = kFalse;
    public Trigger leftStickX = kFalse;
    public Trigger rightStickY = kFalse;
    public Trigger rightStickX = kFalse;

    // Setup function bumper and trigger buttons
    public Trigger noBumpers = rightBumper.negate().and(leftBumper.negate());
    public Trigger leftBumperOnly = leftBumper.and(rightBumper.negate());
    public Trigger rightBumperOnly = rightBumper.and(leftBumper.negate());
    public Trigger bothBumpers = rightBumper.and(leftBumper);
    public Trigger noTriggers = leftTrigger.negate().and(rightTrigger.negate());
    public Trigger leftTriggerOnly = leftTrigger.and(rightTrigger.negate());
    public Trigger rightTriggerOnly = rightTrigger.and(leftTrigger.negate());
    public Trigger bothTriggers = leftTrigger.and(rightTrigger);
    public Trigger noModifiers = noBumpers.and(noTriggers);

    private Rotation2d storedLeftStickDirection = new Rotation2d();
    private Rotation2d storedRightStickDirection = new Rotation2d();
    private boolean configured = false; // Used to determine if we detected the gamepad is plugged and we have
    // configured
    // it
    private boolean printed = false; // Used to only print Gamepad Not Detected once

    @Getter
    protected final ExpCurve leftStickCurve;

    @Getter
    protected final ExpCurve rightStickCurve;

    @Getter
    protected final ExpCurve triggersCurve;

    protected Trigger teleop = RobotModeTriggers.teleop();
    protected Trigger autoMode = RobotModeTriggers.autonomous();
    protected Trigger testMode = RobotModeTriggers.test();
    protected Trigger disabled = RobotModeTriggers.disabled();

    /** Constructs a Gamepad object with the specified configuration. */
    protected Gamepad(
            int port,
            ExpCurve leftStickCurve,
            double leftStickDeadzone,
            ExpCurve rightStickCurve,
            double rightStickDeadzone,
            ExpCurve triggersCurve,
            double triggersDeadzone) {
        disconnectedAlert = new Alert(getName() + " Gamepad Disconnected", Alert.AlertType.kError);

        // Curve objects that we use to configure the controller axis objects
        this.leftStickCurve = leftStickCurve;
        this.rightStickCurve = rightStickCurve;
        this.triggersCurve = triggersCurve;

        xboxController = new CommandXboxController(port);
        A = xboxController.a();
        B = xboxController.b();
        X = xboxController.x();
        Y = xboxController.y();
        leftBumper = xboxController.leftBumper();
        rightBumper = xboxController.rightBumper();
        leftTrigger = xboxController.leftTrigger(triggersDeadzone); // Assuming a default threshold of 0.5
        rightTrigger = xboxController.rightTrigger(triggersDeadzone); // Assuming a default threshold of 0.5
        leftStickClick = xboxController.leftStick();
        rightStickClick = xboxController.rightStick();
        start = xboxController.start();
        select = xboxController.back();
        upDpad = xboxController.povUp();
        downDpad = xboxController.povDown();
        leftDpad = xboxController.povLeft();
        rightDpad = xboxController.povRight();
        leftStickY = leftYTrigger(Threshold.ABS_GREATER, leftStickDeadzone);
        leftStickX = leftXTrigger(Threshold.ABS_GREATER, leftStickDeadzone);
        rightStickY = rightYTrigger(Threshold.ABS_GREATER, rightStickDeadzone);
        rightStickX = rightXTrigger(Threshold.ABS_GREATER, rightStickDeadzone);
        leftStickY = leftYTrigger(Threshold.ABS_GREATER, leftStickDeadzone);
        leftStickX = leftXTrigger(Threshold.ABS_GREATER, leftStickDeadzone);
        rightStickY = rightYTrigger(Threshold.ABS_GREATER, rightStickDeadzone);
        rightStickX = rightXTrigger(Threshold.ABS_GREATER, rightStickDeadzone);
    }

    @Override
    public void periodic() {
        configure();
    }

    // Configure the pilot controller
    public void configure() {
        disconnectedAlert.set(!isConnected()); // Display if the controller is disconnected

        // Detect whether the Xbox controller has been plugged in after start-up
        if (!configured) {
            if (!isConnected()) {
                if (!printed) {
                    Telemetry.print("##" + getName() + ": GAMEPAD NOT CONNECTED ##");
                    printed = true;
                }
                return;
            }

            configured = true;
            Telemetry.print("## " + getName() + ": gamepad is connected ##");
        }
    }

    // Reset the controller configure, should be used with
    // CommandScheduler.getInstance.clearButtons()
    // to reset buttons
    public void resetConfig() {
        configured = false;
        configure();
    }

    /* Zero is stick up, 90 is stick to the left */
    public Rotation2d getLeftStickDirection() {
        double x = -1 * getLeftX();
        double y = -1 * getLeftY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedLeftStickDirection = angle;
        }
        return storedLeftStickDirection;
    }

    public Rotation2d getRightStickDirection() {
        double x = getRightX();
        double y = getRightY();
        if (x != 0 || y != 0) {
            Rotation2d angle = new Rotation2d(y, x);
            storedRightStickDirection = angle;
        }
        return storedRightStickDirection;
    }

    public double getLeftStickCardinals() {
        double stickAngle = getLeftStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    public double getRightStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 4 && stickAngle <= Math.PI / 4) {
            return 0;
        } else if (stickAngle > Math.PI / 4 && stickAngle <= 3 * Math.PI / 4) {
            return Math.PI / 2;
        } else if (stickAngle > 3 * Math.PI / 4 || stickAngle <= -3 * Math.PI / 4) {
            return Math.PI;
        } else {
            return -Math.PI / 2;
        }
    }

    public double getLeftStickMagnitude() {
        double x = -1 * getLeftX();
        double y = -1 * getLeftY();
        return Math.sqrt(x * x + y * y);
    }

    public double getRightStickMagnitude() {
        double x = getRightX();
        double y = getRightY();
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Get proper stick angles for each alliance
     *
     * @return
     */
    public double chooseCardinalDirections() {
        // hotfix
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return getRedAllianceStickCardinals();
        }
        return getBlueAllianceStickCardinals();
    }

    public double getBlueAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();
        if (stickAngle > -Math.PI / 8 && stickAngle <= Math.PI / 8) {
            return 0;
        } else if (stickAngle > Math.PI / 8 && stickAngle <= 3 * Math.PI / 8) {
            return Math.PI / 4;
        } else if (stickAngle > 3 * Math.PI / 8 && stickAngle <= 5 * Math.PI / 8) {
            return Math.PI / 2;
        } else if (stickAngle > 5 * Math.PI / 8 && stickAngle <= 7 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        } // other half of circle
        else if (stickAngle < -Math.PI / 8 && stickAngle >= -3 * Math.PI / 8) {
            return -Math.PI / 4;
        } else if (stickAngle < -3 * Math.PI / 8 && stickAngle >= -5 * Math.PI / 8) {
            return -Math.PI / 2;
        } else if (stickAngle < -5 * Math.PI / 8 && stickAngle >= -7 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        } else {
            return Math.PI; // greater than 7 * Math.PI / 8 or less than -7 * Math.PI / 8 (bottom of
            // circle)
        }
    }

    /**
     * Flips the stick direction for the red alliance.
     *
     * @return
     */
    public double getRedAllianceStickCardinals() {
        double stickAngle = getRightStickDirection().getRadians();

        if (stickAngle > -Math.PI / 8 && stickAngle <= Math.PI / 8) {
            return Math.PI;
        } else if (stickAngle > Math.PI / 8 && stickAngle <= 3 * Math.PI / 8) {
            return -3 * Math.PI / 4;
        } else if (stickAngle > 3 * Math.PI / 8 && stickAngle <= 5 * Math.PI / 8) {
            return -Math.PI / 2;
        } else if (stickAngle > 5 * Math.PI / 8 && stickAngle <= 7 * Math.PI / 8) {
            return -Math.PI / 4;
        } // other half of circle
        else if (stickAngle < -Math.PI / 8 && stickAngle >= -3 * Math.PI / 8) {
            return 3 * Math.PI / 4;
        } else if (stickAngle < -3 * Math.PI / 8 && stickAngle >= -5 * Math.PI / 8) {
            return Math.PI / 2;
        } else if (stickAngle < -5 * Math.PI / 8 && stickAngle >= -7 * Math.PI / 8) {
            return Math.PI / 4;
        } else {
            return 0; // greater than 7 * Math.PI / 8 or less than -7 * Math.PI / 8 (bottom of
            // circle)
        }
    }

    public Trigger leftYTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getLeftY);
    }

    public Trigger leftXTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getLeftX);
    }

    public Trigger rightYTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getRightY);
    }

    public Trigger rightXTrigger(Threshold t, double threshold) {
        return axisTrigger(t, threshold, this::getRightX);
    }

    public Trigger rightStick(double threshold) {
        return new Trigger(() -> Math.abs(getRightX()) >= threshold || Math.abs(getRightY()) >= threshold);
    }

    public Trigger leftStick(double threshold) {
        return new Trigger(() -> Math.abs(getLeftX()) >= threshold || Math.abs(getLeftY()) >= threshold);
    }

    private Trigger axisTrigger(Threshold t, double threshold, DoubleSupplier v) {
        return new Trigger(() -> {
            double value = v.getAsDouble();
            switch (t) {
                case GREATER:
                    return value > threshold;
                case LESS:
                    return value < threshold;
                case ABS_GREATER: // Also called Deadband
                    return Math.abs(value) > threshold;
                default:
                    return false;
            }
        });
    }

    public enum Threshold {
        GREATER,
        LESS,
        ABS_GREATER;
    }

    public void rumble(double leftIntensity, double rightIntensity) {
        rumbleController(leftIntensity, rightIntensity);
    }

    /** Command that can be used to rumble the pilot controller */
    public Command rumbleCommand(double leftIntensity, double rightIntensity, double durationSeconds) {
        return new RunCommand(() -> rumble(leftIntensity, rightIntensity), this)
                .withTimeout(durationSeconds)
                .ignoringDisable(true)
                .withName("Gamepad.Rumble");
    }

    public Command rumbleCommand(double intensity, double durationSeconds) {
        return rumbleCommand(intensity, intensity, durationSeconds);
    }

    /**
     * Returns a new Command object that combines the given command with a rumble command. The rumble command has a
     * rumble strength of 1 and a duration of 0.5 seconds. The name of the returned command is set to the name of the
     * given command.
     *
     * @param command the command to be combined with the rumble command
     * @return a new Command object with rumble command
     */
    public Command rumbleCommand(Command command) {
        return command.alongWith(rumbleCommand(1, 0.5)).withName(command.getName());
    }

    public boolean isConnected() {
        return this.getHID().isConnected();
    }

    protected double getRightTriggerAxis() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightTriggerAxis();
    }

    protected double getLeftTriggerAxis() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftTriggerAxis();
    }

    protected double getTwist() {
        double right = getRightTriggerAxis();
        double left = getLeftTriggerAxis();
        double value = right - left;
        return value;
    }

    protected double getLeftX() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftX();
    }

    protected double getLeftY() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftY();
    }

    protected double getRightX() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightX();
    }

    protected double getRightY() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightY();
    }

    protected GenericHID getHID() {
        return xboxController.getHID();
    }

    protected GenericHID getRumbleHID() {
        if (!isConnected()) {
            return null;
        }
        return xboxController.getHID();
    }

    public void rumbleController(double leftIntensity, double rightIntensity) {
        if (!isConnected()) {
            return;
        }
        getRumbleHID().setRumble(RumbleType.kLeftRumble, leftIntensity);
        getRumbleHID().setRumble(RumbleType.kRightRumble, rightIntensity);
    }
}
