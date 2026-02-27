package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.RobotConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;
import com.revrobotics.spark.SparkLowLevel;

public class Turret extends SubsystemBase {
    private Swivel swivel;
    private Shooter shooter;
    private CommandSwerveDrivetrain drivetrain;
    private Dashboard dashboard;

    public Turret(CommandSwerveDrivetrain drivetrain, Dashboard dashboard) {
        swivel = new Swivel(this);
        shooter = new Shooter(this);
        this.dashboard = dashboard;
        this.drivetrain = drivetrain;
    }

    public Command setShooterSpeed(LinearVelocity speed){
        return shooter.setVelocity(speed);
    }
    public Command set(double dutyCycle){
        return shooter.set(dutyCycle);
    }

    @Override
    public void periodic() {
        swivel.updateTelemetry();
        swivel.runSetPoint();
        shooter.periodic();
        
    }

    @Override
    public void simulationPeriodic() {
        swivel.simIterate();
        shooter.simulationPeriodic();
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
        SmartMotorControllerConfig smartMotorControllerConfig;
        private SparkMax flywheelMotor;
        private SmartMotorController motor;
        private FlyWheelConfig fConfig;
        private FlyWheel shooter;
        public Shooter(Turret turret) {
            smartMotorControllerConfig = new SmartMotorControllerConfig(turret)
                .withControlMode(ControlMode.CLOSED_LOOP)
                // Feedback Constants (PID Constants)
                .withClosedLoopController(.01, 0, 0 )
                // Telemetry name and verbosity level
                .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
                // Gearing from the motor rotor to final shaft.
                // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
                // You could also use .withGearing(12) which does the same thing.
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                // Motor properties to prevent over currenting.
                .withMotorInverted(false)
                .withIdleMode(MotorMode.COAST)
                .withStatorCurrentLimit(Amps.of(40))
                .withFollowers(Pair.of(new SparkMax(0, MotorType.kBrushless), true));
                
                flywheelMotor = new SparkMax(30, MotorType.kBrushless);
                motor = new SparkWrapper(flywheelMotor, DCMotor.getNEO(2), smartMotorControllerConfig);

                fConfig = new FlyWheelConfig(motor)
                // Diameter of the flywheel.
                .withDiameter(Inches.of(4))
                // Mass of the flywheel.
                .withMass(Pounds.of(1))
                // Telemetry name and verbosity for the arm.
                .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
                .disableSpeedometerSimulation();
                shooter = new FlyWheel(fConfig);

            }
        
        public Command setVelocity(LinearVelocity speed) {
            return run(() ->shooter.setMeasurementVelocitySetpoint(speed));
        }
        public Command set(double dutyCycle) {
            return shooter.set(dutyCycle);
        }

        private AngularVelocity getVelocity(){
            return shooter.getSpeed();
        }
        
  
        private Boolean shooterReady() {
            return true;// TODO: Add check to see if shooter meets conditions to fire
        }
        
       
        public void periodic() {
            // This method will be called once per scheduler run
            shooter.updateTelemetry();
            SmartDashboard.putNumber("Shooter Speed", getVelocity().in(RPM));
        }

        public void simulationPeriodic() {
            // This method will be called once per scheduler run during simulation
            shooter.simIterate();
            }
}
}