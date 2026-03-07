package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dashboard;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
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


public class Swivel extends SubsystemBase {
    private static Swivel swivel =null;

    public static Swivel getInstance(){
        if ( swivel == null){
            swivel = new Swivel();
        }
        return swivel;
    }
    
    Dashboard dashboard = Dashboard.getInstance();
    CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
              private PIDController turretController;
                private TalonFX turretMotor;
                private SmartMotorControllerConfig motorConfig;
                private SmartMotorController motor;
                private PivotConfig pConfig;
                private Pivot pivot;
                Slot0Configs Slot0Configs = new Slot0Configs()
                    .withKA(RobotConstants.Turret.TURRET_MAX_ACC.in(RotationsPerSecondPerSecond))
                    .withKV(RobotConstants.Turret.TURRET_MAX_VEL.in(RotationsPerSecond))
                    .withKD(RobotConstants.Turret.TURRET_KD)
                    .withKI(RobotConstants.Turret.TURRET_KI)
                    .withKP(RobotConstants.Turret.TURRET_KP);                                                                                                                
        
                private Swivel() {
                    turretController = new PIDController(RobotConstants.Turret.TURRET_KP, RobotConstants.Turret.TURRET_KI,
                    RobotConstants.Turret.TURRET_KD);
                    turretController.disableContinuousInput();
                    turretMotor = new TalonFX(RobotConstants.Turret.TURRET_CAN_ID);
                    motorConfig = new SmartMotorControllerConfig(this)
                            .withControlMode(ControlMode.CLOSED_LOOP)
                            .withClosedLoopController(RobotConstants.Turret.TURRET_KP, RobotConstants.Turret.TURRET_KI,
                                    RobotConstants.Turret.TURRET_KD, RobotConstants.Turret.TURRET_MAX_VEL,
                                    RobotConstants.Turret.TURRET_MAX_ACC)
                            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3,3,7)))
                            .withIdleMode(MotorMode.BRAKE)
                            .withMotorInverted(false)
                            .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                            .withStatorCurrentLimit(Amps.of(40))
                            .withClosedLoopRampRate(Seconds.of((.25)))
                            .withOpenLoopRampRate(Seconds.of((.25)));

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

        public Command runSetPoint(Angle angle) {
            return runOnce(() -> pivot.setMechanismPositionSetpoint(angle));
        }

        public Command setdutyCycle(double dutyCycle){
            return run(() -> pivot.setDutyCycleSetpoint(dutyCycle));
        }

        private Boolean targeted() {
            return true;// TODO: add check to see if pivot is at the setpoint
        }
    }



