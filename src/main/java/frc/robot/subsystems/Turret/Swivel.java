package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import frc.robot.subsystems.CommandSwerveDrivetrain;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Swivel extends SubsystemBase {
    private static Swivel swivel = null;

    private TurretHelper turretHelper = TurretHelper.getInstance();
    private CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private PIDController SwivelController;
    private TalonFX SwivelMotor;
    private SmartMotorControllerConfig motorConfig;
    private SmartMotorController motor;
    private PivotConfig pConfig;
    private Pivot pivot;

    public static Swivel getInstance() {
        if (swivel == null) {
            swivel = new Swivel();
        }
        return swivel;
    }

    private Swivel() {
        SwivelController = new PIDController(RobotConstants.SwivelConstants.SWIVEL_KP,
                RobotConstants.SwivelConstants.SWIVEL_KI,
                RobotConstants.SwivelConstants.SWIVEL_KD);
        SwivelController.disableContinuousInput();
        SwivelMotor = new TalonFX(RobotConstants.SwivelConstants.SWIVEL_CAN_ID);
        motorConfig = new SmartMotorControllerConfig(this)
                .withControlMode(ControlMode.CLOSED_LOOP)
                .withClosedLoopController(RobotConstants.SwivelConstants.SWIVEL_KP,
                        RobotConstants.SwivelConstants.SWIVEL_KI,
                        RobotConstants.SwivelConstants.SWIVEL_KD, RobotConstants.SwivelConstants.SWIVEL_MAX_VEL,
                        RobotConstants.SwivelConstants.SWIVEL_MAX_ACC)
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 3, 6.75)))
                .withIdleMode(MotorMode.BRAKE)
                .withMotorInverted(false)
                .withTelemetry("SwivelMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(Amps.of(40))
                .withClosedLoopRampRate(Seconds.of((.25)))
                .withOpenLoopRampRate(Seconds.of((.25)));

        motor = new TalonFXWrapper(SwivelMotor, DCMotor.getKrakenX60(1), motorConfig);

        pConfig = new PivotConfig(motor)
                .withStartingPosition(Degrees.of(0))
                .withTelemetry("SwivelPivot", TelemetryVerbosity.HIGH)
                .withHardLimit(Degrees.of(-180), Degrees.of(180))
                .withMOI(Inches.of(4), Pounds.of(2.72));
        pivot = new Pivot(pConfig);
    }

    public Angle manuelSwivelAngle() {
        double setAngle = SmartDashboard.getNumber("Set Swivel Angle", 0);
        return Angle.ofBaseUnits(setAngle, Degrees);

    }

    public void runSetPoint(Angle angle) {
        pivot.setMechanismPositionSetpoint(angle);
    }

    public Angle getAngle() {
        return pivot.getAngle();
    }

    public Command setdutyCycle(double dutyCycle) {
        return run(() -> pivot.setDutyCycleSetpoint(dutyCycle));
    }

    private Boolean targeted() {
        return true;// TODO: add check to see if pivot is at the setpoint
    }

    @Override
    public void periodic() {
        pivot.updateTelemetry();

    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }
}
