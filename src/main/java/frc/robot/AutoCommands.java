package frc.robot;

import java.io.IOException;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommands {
    public AutoCommands() {
    }

    public static Command createPath() {
        // Load the path we want to pathfind to and follow
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile("Right");
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
            return null;
        }

        // Create the constraints to use while pathfinding. The constraints defined in
        // the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints);

        return pathfindingCommand;
    }
}
