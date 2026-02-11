package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.fasterxml.jackson.databind.node.ShortNode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
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
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret extends SubsystemBase {
    private PIDController turretController;
    private AnalogEncoder analogEncoder;
    private TalonFX turretMotor;
    private SmartMotorControllerConfig motorConfig;
    private SmartMotorController motor;
    private PivotConfig pConfig;
    private Pivot pivot;
    private CommandSwerveDrivetrain drivetrain;

    public Turret(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        turretController = new PIDController(RobotConstants.Turret.TURRET_KP, RobotConstants.Turret.TURRET_KI,
                RobotConstants.Turret.TURRET_KD);
        turretController.disableContinuousInput();
        analogEncoder = new AnalogEncoder(RobotConstants.Turret.ENCODER_ID, 10.0, 0);
        turretMotor = new TalonFX(RobotConstants.Turret.TURRET_CAN_ID);

        motorConfig = new SmartMotorControllerConfig(this)
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
        // .withExternalEncoder(analogEncoder);
        motor = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

        pConfig = new PivotConfig(motor)
                .withStartingPosition(Degrees.of(0))
                .withTelemetry("TurretPivot", TelemetryVerbosity.HIGH)
                .withHardLimit(Degrees.of(-180), Degrees.of(180))
                .withMOI(Inches.of(4), Pounds.of(2.72));
        pivot = new Pivot(pConfig);
    }

    @Override
    public void periodic() {
        pivot.updateTelemetry();
        SmartDashboard.putNumber("Angle from pivot", pivot.getAngle().in(Degrees));
        pivot.setMechanismPositionSetpoint(getTurretSetpoint());
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }

    public Angle getTurretSetpoint() {
        Rotation2d aimAngle = new Rotation2d(Math.atan2(getTarget().getY(), getTarget().getX()));
        return aimAngle.minus(drivetrain.getPigeon2().getRotation2d()).getMeasure();
    }

    private Translation2d getTarget() {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            if (drivetrain.getState().Pose.getX() < RobotConstants.Turret.HUB_X) {
                return RobotConstants.Turret.HUB.minus(drivetrain.getState().Pose.getTranslation());
            }
            if (drivetrain.getState().Pose.getY() >= RobotConstants.Turret.HUB_Y) {
                return RobotConstants.Turret.UPPER_TRENCH.minus(drivetrain.getState().Pose.getTranslation());
            }
            return RobotConstants.Turret.LOWER_TRENCH.minus(drivetrain.getState().Pose.getTranslation());
        }
        if (drivetrain.getState().Pose.getX() > RobotConstants.Turret.HUB_X) {
            return RobotConstants.Turret.HUB.minus(drivetrain.getState().Pose.getTranslation());
        }
        if (drivetrain.getState().Pose.getY() >= RobotConstants.Turret.HUB_Y) {
            return RobotConstants.Turret.UPPER_TRENCH.minus(drivetrain.getState().Pose.getTranslation());
        }
        return RobotConstants.Turret.LOWER_TRENCH.minus(drivetrain.getState().Pose.getTranslation());
    }
}
