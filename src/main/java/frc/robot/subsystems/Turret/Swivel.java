package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Degrees;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Swivel extends SubsystemBase {

    private TalonFX motor;
    private TalonFXConfiguration motorConfig;
    private AnalogPotentiometer encoder;
    private CommandSwerveDrivetrain drivetrain;
    private Dashboard dashboard;

    private static Swivel swivel = null;

    public static Swivel getInstance() {
        if (swivel == null) {
            swivel = new Swivel();
        }
        return swivel;
    }

    public Swivel() {
        dashboard = Dashboard.getInstance();
        drivetrain = CommandSwerveDrivetrain.getInstance();
        motor = new TalonFX(RobotConstants.Turret.TURRET_CAN_ID);
        motorConfig = new TalonFXConfiguration();
        motorConfig.ClosedLoopGeneral.withContinuousWrap(false);
        motorConfig.Feedback.withSensorToMechanismRatio(0);
        motorConfig.Slot0.withKP(RobotConstants.Turret.TURRET_KP).withKI(RobotConstants.Turret.TURRET_KI)
                .withKD(RobotConstants.Turret.TURRET_KD);
        motor.getConfigurator().apply(motorConfig);
        encoder = new AnalogPotentiometer(0, 514, -257);
        motor.getConfigurator().setPosition(Degrees.of(encoder.get()));
    }

    private Angle getTurretSetpoint() {
        Rotation2d aimAngle = dashboard.shootAngle();
        return aimAngle.minus(drivetrain.getPigeon2().getRotation2d()).getMeasure();
    }

    public Command runSwivel() {
        return Commands.run(() -> motor.setControl(new PositionVoltage(getTurretSetpoint())));
    }

    public Command setDutyCycle(double dutyCycle) {
        return Commands.run(() -> motor.set(dutyCycle), this);
    }

    private Boolean targeted() {
        return true;// TODO: add check to see if pivot is at the setpoint
    }
}