package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret.Loader;
import frc.robot.subsystems.Turret.Shooter;
import frc.robot.subsystems.Turret.Swivel;
import frc.robot.subsystems.Turret.TurretHelper;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CompositeCommands {
    static Intake intake = Intake.getInstance();
    static Loader loader = Loader.getInstance();
    static Shooter shooter = Shooter.getInstance();
    static Swivel swivel = Swivel.getInstance();
    static SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric();
    static TurretHelper turretHelper = TurretHelper.getInstance();

    static CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

    public CompositeCommands() {

    }

    public static Command runLoader() {
        return Commands.sequence(Loader.getInstance().runOnce(() -> Loader.getInstance().setLoader(-.1)),
                Commands.waitSeconds(.1),
                Loader.getInstance().run(() -> Loader.getInstance().setLoader(.2)));
    }

    public static Command intakeOn() {
        return Commands.sequence(intake.runOnce(() -> intake.pushIntakeOut()),
                intake.runOnce(() -> intake.setVacuum(.3)));
    }

    public static Command intakeOff() {
        return Commands.sequence(intake.runOnce(() -> intake.setVacuum(0)),
                intake.runOnce(() -> intake.pullIntakeIn()));
    }

    public static Command shakeBot() {
        return Commands.repeatingSequence(
                drivetrain.applyRequest(() -> robotDrive.withVelocityX(MetersPerSecond.of(-5))).withTimeout(0.1),
                drivetrain.applyRequest(() -> robotDrive.withVelocityX(MetersPerSecond.of(5))).withTimeout(0.12));
    }

    public static Command swivelShoot() {
        return swivel.run(() -> swivel.runSetPoint(turretHelper.shootAngle().getMeasure()))
                .alongWith(shooter.run(() -> shooter
                        .setVelocity(turretHelper.launchSpeed())));
    }

}
