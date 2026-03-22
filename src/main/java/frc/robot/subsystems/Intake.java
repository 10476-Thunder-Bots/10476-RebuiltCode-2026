package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Intake extends SubsystemBase {
    private static Intake intake = null;

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }

    private SparkFlex motor;
    private SparkFlexConfig motorConfig;
    private SparkFlex motorFollower;
    private SparkFlexConfig motorFollowerConfig;
    private TalonFX motorKraken;
    private TalonFXConfiguration motorKrakenConfig;
    private Slot0Configs slot0Configs;

    private Intake() {
        motor = new SparkFlex(19, MotorType.kBrushless);
        motorConfig = new SparkFlexConfig();
        motorConfig.inverted(false);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorFollower = new SparkFlex(20, MotorType.kBrushless);
        motorFollowerConfig = new SparkFlexConfig();
        motorFollowerConfig.follow(motor, true);
        motorFollower.configure(motorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorKraken = new TalonFX(21);
        slot0Configs = new Slot0Configs()
                .withKP(RobotConstants.IntakeConstants.Intake_KP)
                .withKD(RobotConstants.IntakeConstants.Intake_KD)
                .withKI(RobotConstants.IntakeConstants.Intake_KI);
        motorKrakenConfig = new TalonFXConfiguration().withSlot0(slot0Configs);
        motorKrakenConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
        motorKraken.getConfigurator().apply(motorKrakenConfig);

    }

    public void setVacuum(double speed) {
        motor.set(speed);
    }

    public void setIntakeMover() {
        motorKraken.set(.1);
    }

    public void pushIntakeOut() {
        motorKraken.setPosition(Degrees.of(0));
    }

    public void pullIntakeIn() {
        motorKraken.setPosition(Degrees.of(0));
    }

    public Angle getIntakePosition() {
        return motorKraken.getPosition().getValue();
    }
}
