package org.supurdueper.lib.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.ArrayList;
import java.util.List;
import org.supurdueper.lib.LoggedTunableNumber;

public abstract class PositionSubsystem extends TalonFXSubsystem {

    // Tunable numbers
    private final LoggedTunableNumber kp;
    private final LoggedTunableNumber ki;
    private final LoggedTunableNumber kd;
    private final LoggedTunableNumber ks;
    private final LoggedTunableNumber kv;
    private final LoggedTunableNumber ka;
    private final LoggedTunableNumber kg;
    private final LoggedTunableNumber profileKv;
    private final LoggedTunableNumber profileKa;
    private final List<LoggedTunableNumber> pidGains;
    private final GravityTypeValue gravityTypeValue;
    private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);
    private final Angle positionTolerance;
    private final SysIdRoutine sysIdRoutine;

    public Command sysIdQuasistaticFoward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }

    public Command sysIdDynamicFoward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }

    public Command goToPosition(Angle rotations) {
        return run(() -> setPosition(rotations));
    }

    public Command goToPosition(double rotations) {
        return run(() -> setPosition(rotations));
    }

    public Command goToPositionBlocking(Angle rotations) {
        return goToPosition(rotations).andThen(Commands.waitUntil(this::atPosition));
    }

    public Trigger isAtPosition() {
        return new Trigger(this::atPosition);
    }

    protected void setPosition(Angle position) {
        motor.setControl(positionRequest.withPosition(position));
    }

    protected void setPosition(double position) {
        motor.setControl(positionRequest.withPosition(position));
    }

    protected Angle getPosition() {
        return motor.getPosition().getValue();
    }

    protected Angle getSetpoint() {
        return Units.Rotations.of(motor.getClosedLoopReference().getValueAsDouble());
    }

    protected boolean atPosition() {
        return (getSetpoint().minus(getPosition())).abs(Units.Rotations) < (positionTolerance.in(Units.Rotations));
    }

    public PositionSubsystem() {
        super();

        // Setup tunable pid gains
        String name = this.getName();
        kp = new LoggedTunableNumber(name + "/Kp");
        ki = new LoggedTunableNumber(name + "/Ki");
        kd = new LoggedTunableNumber(name + "/Kd");
        ks = new LoggedTunableNumber(name + "/Ks");
        kv = new LoggedTunableNumber(name + "/Kv");
        ka = new LoggedTunableNumber(name + "/Ka");
        kg = new LoggedTunableNumber(name + "/Kg");
        profileKv = new LoggedTunableNumber(name + "/profileKv");
        profileKa = new LoggedTunableNumber(name + "/profileKa");
        Slot0Configs gains = pidGains();
        gravityTypeValue = gains.GravityType;
        kp.initDefault(gains.kP);
        ki.initDefault(gains.kI);
        kd.initDefault(gains.kD);
        ks.initDefault(gains.kS);
        kv.initDefault(gains.kV);
        ka.initDefault(gains.kA);
        kg.initDefault(gains.kG);
        MotionMagicConfigs motionMagicConfig = motionMagicConfig();
        profileKv.initDefault(motionMagicConfig.MotionMagicExpo_kV);
        profileKa.initDefault(motionMagicConfig.MotionMagicExpo_kA);
        pidGains = new ArrayList<>();
        pidGains.addAll(List.of(kp, ki, kd, ks, kv, ka, kg, profileKa, profileKv));
        positionTolerance = positionTolerance();
        sysIdRoutine = sysIdConfig();
        // Add motion magic items to config
        config = config.withSlot0(gains).withMotionMagic(motionMagicConfig).withSoftwareLimitSwitch(softLimitConfig());
    }

    @Override
    public void periodic() {
        for (LoggedTunableNumber gain : pidGains) {
            if (gain.hasChanged(hashCode())) {
                // Send new PID gains to talon
                Slot0Configs slot0config = new Slot0Configs()
                        .withGravityType(gravityTypeValue)
                        .withKP(kp.get())
                        .withKI(ki.get())
                        .withKD(kd.get())
                        .withKS(ks.get())
                        .withKV(kv.get())
                        .withKA(ka.get())
                        .withKG(kg.get());
                MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                        .withMotionMagicExpo_kA(profileKa.get())
                        .withMotionMagicExpo_kV(profileKv.get());
                motor.getConfigurator().apply(config.withSlot0(slot0config).withMotionMagic(motionMagicConfigs));
                break;
            }
        }
        super.periodic();
    }

    public abstract Slot0Configs pidGains();

    public abstract MotionMagicConfigs motionMagicConfig();

    public abstract SoftwareLimitSwitchConfigs softLimitConfig();

    public abstract Angle positionTolerance();

    public abstract SysIdRoutine sysIdConfig();
}
