package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Turret.Intake;

public class CompositeCommands {
    private static Intake intake = Intake.getInstance();

    public CompositeCommands() {
    }

    public static Command exampleCommand() {
        return Commands.sequence(exampleCommand(), exampleCommand(), Commands.waitTime(Seconds.of(5)));
    }

    public static Command runIntake() {
        return Commands.sequence( intake.setIntake(.2));
    }
}
