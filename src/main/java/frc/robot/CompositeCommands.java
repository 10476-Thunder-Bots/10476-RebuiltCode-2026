package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Turret.Intake;

public class CompositeCommands {
    private static Intake intake = Intake.getInstance();

    public CompositeCommands() {
    }

    public static Command runIntake() {
        return intake.runOnce(
                () -> intake.setIntake(-.2).andThen(Commands.waitSeconds(.1).finallyDo(() -> intake.setIntake(.2))));
    }
}
