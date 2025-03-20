package org.supurdueper.robot2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSysId {

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation;

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer;

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation;

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply;

    public DriveSysId(Drivetrain drive) {
        m_sysIdRoutineTranslation = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> drive.setControl(m_translationCharacterization.withVolts(output)), null, drive));
        m_sysIdRoutineSteer = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(7), // Use dynamic voltage of 7 V
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
                new SysIdRoutine.Mechanism(
                        volts -> drive.setControl(m_steerCharacterization.withVolts(volts)), null, drive));
        m_sysIdRoutineRotation = new SysIdRoutine(
                new SysIdRoutine.Config(
                        /* This is in radians per second², but SysId only supports "volts per second" */
                        Volts.of(Math.PI / 6).per(Second),
                        /* This is in radians per second, but SysId only supports "volts" */
                        Volts.of(Math.PI),
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> {
                            /* output is actually radians per second, but SysId only supports "volts" */
                            drive.setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                            /* also log the requested output for SysId */
                            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                        },
                        null,
                        drive));
        m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine specified by
     * {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /** SysId-specific SwerveRequest to characterize the translational characteristics of a swerve drivetrain. */
    public static class SysIdSwerveTranslationCurrent implements SwerveRequest {
        /** Voltage to apply to drive wheels. */
        public double CurrentToApply = 0;

        /** Local reference to a voltage request for the drive motors */
        private final TorqueCurrentFOC m_driveRequest = new TorqueCurrentFOC(0);
        /** Local reference to a position voltage request for the steer motors */
        private final PositionVoltage m_steerRequest_Voltage = new PositionVoltage(0);
        /** Local reference to a position torque current request for the steer motors */
        private final PositionTorqueCurrentFOC m_steerRequest_TorqueCurrent = new PositionTorqueCurrentFOC(0);

        public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
            for (int i = 0; i < modulesToApply.length; ++i) {
                switch (modulesToApply[i].getSteerClosedLoopOutputType()) {
                    case Voltage:
                        modulesToApply[i].apply(
                                m_driveRequest.withOutput(CurrentToApply), m_steerRequest_Voltage.withPosition(0));
                        break;
                    case TorqueCurrentFOC:
                        modulesToApply[i].apply(
                                m_driveRequest.withOutput(CurrentToApply),
                                m_steerRequest_TorqueCurrent.withPosition(0));
                        break;
                }
            }
            return StatusCode.OK;
        }

        /**
         * Sets the voltage to apply to the drive wheels.
         *
         * @param volts Voltage to apply
         * @return this request
         */
        public SysIdSwerveTranslationCurrent withCurrent(double volts) {
            CurrentToApply = volts;
            return this;
        }
        /**
         * Sets the voltage to apply to the drive wheels.
         *
         * @param volts Voltage to apply
         * @return this request
         */
        public SysIdSwerveTranslationCurrent withCurrent(Voltage volts) {
            CurrentToApply = volts.in(Volts);
            return this;
        }
    }
}
