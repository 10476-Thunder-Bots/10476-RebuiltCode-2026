package frc.robot.subsystems.Turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private SparkFlex motor;
    private SparkFlex motorFollower;
    private SparkFlexConfig motorConfig;
    private SparkFlexConfig followerConfig;

    private static Intake intake = null;

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }

    private Intake() {
        motor = new SparkFlex(17, MotorType.kBrushless);
        motorFollower = new SparkFlex(18, MotorType.kBrushless);
        motorConfig = new SparkFlexConfig();
        motorConfig.inverted(false);
        followerConfig = new SparkFlexConfig();
        followerConfig.follow(17, true);
        motorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command setIntake(double speed) {
        return run(() -> motor.set(speed));
    }
}
