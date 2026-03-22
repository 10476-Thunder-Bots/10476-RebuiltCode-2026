package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret.Loader;
import frc.robot.subsystems.Turret.Shooter;
import frc.robot.subsystems.Turret.Swivel;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CompositeCommands {
    static Intake intake = Intake.getInstance();
    static Loader loader = Loader.getInstance();
    static Shooter shooter = Shooter.getInstance();
    static Swivel swivel = Swivel.getInstance();
    static SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric();
        
    
    static CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    public CompositeCommands() {

    }


    public static Command runLoader() {
        return Commands.sequence(Loader.getInstance().runOnce(() -> Loader.getInstance().setLoader(-.1)),
                Commands.waitSeconds(.1),
                Loader.getInstance().run(() -> Loader.getInstance().setLoader(.2)));
    }
    
    public static Command intakeOn(){
        return Commands.sequence(intake.runOnce(() -> intake.pushIntakeOut()), intake.runOnce(() -> intake.setVacuum(.3)));
    }

    public static Command intakeOff(){
        return Commands.sequence(intake.runOnce(() -> intake.setVacuum(0)), intake.runOnce(() -> intake.pullIntakeIn()));
    }

    public static Command shakeBot(){
        return Commands.repeatingSequence(drivetrain.applyRequest(() -> robotDrive.withRotationalRate(Math.toRadians(-270))).withTimeout(0.3),
        drivetrain.applyRequest(() -> robotDrive.withRotationalRate(Math.toRadians(270))).withTimeout(0.32));
}
}
