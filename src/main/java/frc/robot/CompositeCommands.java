package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret.Loader;
import frc.robot.subsystems.Turret.Shooter;
import frc.robot.subsystems.Turret.Swivel;
import frc.robot.subsystems.Turret.TurretHelper;
import yams.mechanisms.swerve.SwerveDrive;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CompositeCommands {
    static Intake intake = Intake.getInstance();
    static Loader loader = Loader.getInstance();
    static Shooter shooter = Shooter.getInstance();
    static Swivel swivel = Swivel.getInstance();
    static SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric();
    static TurretHelper turretHelper = TurretHelper.getInstance();
    static SwerveRequest.Idle idle =new SwerveRequest.Idle();
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
                intake.runOnce(() -> intake.setVacuum(-.5)));
    }

    public static Command intakeOff() {
        return Commands.sequence(intake.runOnce(() -> intake.setVacuum(0)),
                intake.runOnce(() -> intake.pullIntakeIn()));
    }

    public static Command shakeBotX() {
        return Commands.repeatingSequence(drivetrain.applyRequest(()-> robotDrive.withVelocityY(0)).withTimeout(.01), 
        drivetrain.applyRequest(()-> robotDrive.withVelocityX(0)).withTimeout(0.01),
        drivetrain.applyRequest(()->robotDrive.withRotationalRate(AngularVelocity.ofBaseUnits(0, RotationsPerSecond))).withTimeout(.01),
        drivetrain.applyRequest(()-> robotDrive.withVelocityX(LinearVelocity.ofBaseUnits(-5, MetersPerSecond))).withTimeout(0.1),
        drivetrain.applyRequest(()-> robotDrive.withVelocityX(LinearVelocity.ofBaseUnits(5, MetersPerSecond))).withTimeout(0.1));
    }

    public static Command swivelShoot() {
        return swivel.run(() -> swivel.runSetPoint(turretHelper.shootAngle().getMeasure()))
                .alongWith(shooter.run(() -> shooter
                        .setVelocity(turretHelper.launchSpeed())));
    }

      public static Command shakeBotY() { 
        return Commands.repeatingSequence(
        drivetrain.applyRequest(()-> robotDrive.withVelocityX(0)).withTimeout(0.01),
        drivetrain.applyRequest(()-> robotDrive.withVelocityY(0)).withTimeout(0.01),
        drivetrain.applyRequest(()->robotDrive.withRotationalRate(AngularVelocity.ofBaseUnits(0, RotationsPerSecond))).withTimeout(.01),
        drivetrain.applyRequest(()-> robotDrive.withVelocityY(LinearVelocity.ofBaseUnits(-5, MetersPerSecond))).withTimeout(0.1),
        drivetrain.applyRequest(()-> robotDrive.withVelocityY(LinearVelocity.ofBaseUnits(5, MetersPerSecond))).withTimeout(0.1));
      }
      public static Command shakeRotate(){
        return Commands.repeatingSequence(
        drivetrain.applyRequest(()-> robotDrive.withVelocityX(0)).withTimeout(0.01),
        drivetrain.applyRequest(()->robotDrive.withRotationalRate(AngularVelocity.ofBaseUnits(0, RotationsPerSecond))).withTimeout(.01),
        drivetrain.applyRequest(()-> robotDrive.withVelocityY(0)).withTimeout(0.01),
        drivetrain.applyRequest(()->robotDrive.withRotationalRate(AngularVelocity.ofBaseUnits(60, RotationsPerSecond))).withTimeout(.1),
        drivetrain.applyRequest(()->robotDrive.withRotationalRate(AngularVelocity.ofBaseUnits(-60, RotationsPerSecond))).withTimeout(.1)
        );
      } 
      
      public static Command SpinBeybalde(){
        return drivetrain.applyRequest(()->robotDrive.withRotationalRate(
            AngularVelocity.ofBaseUnits(5, RotationsPerSecond)));
      }
 

    public static Command ballUnstucker(){
        return Commands.repeatingSequence(
                intake.runOnce(()-> intake.pushIntakeOut()), Commands.waitSeconds(.5),
                intake.runOnce(()-> intake.pullIntakeIn()), Commands.waitSeconds(.5)
        );
    }
}
