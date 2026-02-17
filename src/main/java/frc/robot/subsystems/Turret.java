package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.RobotConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret extends SubsystemBase {
    private Swivel swivel;
    private CommandSwerveDrivetrain drivetrain;
    private Dashboard dashboard;

    public Turret(CommandSwerveDrivetrain drivetrain, Dashboard dashboard) {
        swivel = new Swivel(this);
        this.dashboard = dashboard;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        swivel.updateTelemetry();
        swivel.runSetPoint();
    }

    @Override
    public void simulationPeriodic() {
        swivel.simIterate();
    }

    private class Swivel {
        private PIDController turretController;
        private TalonFX turretMotor;
        private SmartMotorControllerConfig motorConfig;
        private SmartMotorController motor;
        private PivotConfig pConfig;
        private Pivot pivot;

        private Swivel(Turret turret) {
            turretController = new PIDController(RobotConstants.Turret.TURRET_KP, RobotConstants.Turret.TURRET_KI,
                    RobotConstants.Turret.TURRET_KD);
            turretController.disableContinuousInput();
            turretMotor = new TalonFX(RobotConstants.Turret.TURRET_CAN_ID);
            motorConfig = new SmartMotorControllerConfig(turret)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(RobotConstants.Turret.TURRET_KP, RobotConstants.Turret.TURRET_KI,
                            RobotConstants.Turret.TURRET_KD, RobotConstants.Turret.TURRET_MAX_VEL,
                            RobotConstants.Turret.TURRET_MAX_ACC)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(9)))
                    .withIdleMode(MotorMode.BRAKE)
                    .withMotorInverted(false)
                    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withClosedLoopRampRate(Seconds.of((.25)))
                    .withOpenLoopRampRate(Seconds.of((.25)));
            // .withExternalEncoder(new AnalogEncoder(RobotConstants.Turret.ENCODER_ID,
            // 10.0, 0));
            motor = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);
            pConfig = new PivotConfig(motor)
                    .withStartingPosition(Degrees.of(0))
                    .withTelemetry("TurretPivot", TelemetryVerbosity.HIGH)
                    .withHardLimit(Degrees.of(-180), Degrees.of(180))
                    .withMOI(Inches.of(4), Pounds.of(2.72));
            pivot = new Pivot(pConfig);
        }

        private Angle getTurretSetpoint() {
            Rotation2d aimAngle = dashboard.shootAngle();
            return aimAngle.minus(drivetrain.getPigeon2().getRotation2d()).getMeasure();
        }

        public void updateTelemetry() {
            pivot.updateTelemetry();
        }

        public void simIterate() {
            pivot.simIterate();
        }

        public void runSetPoint() {
            pivot.setMechanismPositionSetpoint(getTurretSetpoint());
        }

        private Boolean targeted() {
            return true;// TODO: add check to see if pivot is at the setpoint
        }
    }

    private class Shooter {
        public Shooter() {
        }

        private Boolean shooterReady() {
            return true;// TODO: Add check to see if shooter meets conditions to fire
        }
    }
}
