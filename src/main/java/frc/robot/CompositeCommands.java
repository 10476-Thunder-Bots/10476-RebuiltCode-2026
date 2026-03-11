package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Turret.Loader;

public class CompositeCommands {

    public CompositeCommands() {
    }

    public static Command runIntake() {
        return Commands.sequence(Loader.getInstance().runOnce(() ->Loader.getInstance().setIntake(-.1)),
                Commands.waitSeconds(.1),
                Loader.getInstance().run(() -> Loader.getInstance().setLoader(.2)));
    }
}
