package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import com.revrobotics.spark.SparkFlex;

public class Shooter extends SubsystemBase {
    private SmartMotorControllerConfig smartMotorControllerConfig;
    private SparkFlex flywheelMotor;
    private SmartMotorController motor;
    private FlyWheelConfig fConfig;
    private FlyWheel flywheel;

    private static Shooter shooter = null;

    public static Shooter getInstance() {
        if (shooter == null) {
            shooter = new Shooter();
        }
        return shooter;
    }

    public Shooter() {
        smartMotorControllerConfig = new SmartMotorControllerConfig(this)
                .withControlMode(ControlMode.CLOSED_LOOP)
                // Feedback Constants (PID Constants)
                .withClosedLoopController(.01, 0, 0)
                // Telemetry name and verbosity level
                .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
                // Gearing from the motor rotor to final shaft.
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                // Motor properties to prevent over currenting.
                .withMotorInverted(false)
                .withIdleMode(MotorMode.COAST)
                .withStatorCurrentLimit(Amps.of(40))
                .withFollowers(Pair.of(new SparkFlex(15, MotorType.kBrushless), true));

        flywheelMotor = new SparkFlex(16, MotorType.kBrushless);
        motor = new SparkWrapper(flywheelMotor, DCMotor.getNeoVortex(2), smartMotorControllerConfig);

        fConfig = new FlyWheelConfig(motor)
                // Diameter of the flywheel.
                .withDiameter(Inches.of(4))
                // Mass of the flywheel.
                .withMass(Pounds.of(5))
                // Telemetry name and verbosity for the arm.
                .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
                .disableSpeedometerSimulation();
        flywheel = new FlyWheel(fConfig);

    }

    public Command setVelocity(LinearVelocity speed) {
        return flywheel.run(speed);
    }

    public Command set(double dutyCycle) {
        return flywheel.set(dutyCycle);
    }

    private LinearVelocity getVelocity() {
        return flywheel.getLinearVelocity();
    }

    private Boolean shooterReady() {
        return true;// TODO: Add check to see if shooter meets conditions to fire
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        flywheel.updateTelemetry();
        SmartDashboard.putNumber("Shooter Speed", getVelocity().in(MetersPerSecond));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        flywheel.simIterate();
    }
}