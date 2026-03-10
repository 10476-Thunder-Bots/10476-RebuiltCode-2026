package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Turret.Intake;

public class CompositeCommands {

    public CompositeCommands() {
    }

    public static Command runIntake() {
        return Commands.sequence(Intake.getInstance().runOnce(() -> Intake.getInstance().setIntake(-.1)),
                Commands.waitSeconds(.1),
                Intake.getInstance().run(() -> Intake.getInstance().setIntake(.2)));
    }
}
