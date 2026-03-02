package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CompositeCommands {
    public static Command exampleCommand(){
        return Commands.run(()->exampleCommand().andThen(exampleCommand().andThen(Commands.waitTime(Seconds.of(5)))));
    }
}
