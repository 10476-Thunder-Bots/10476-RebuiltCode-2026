package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoCommands {
    private Command enterLeft;
    private Command leaveLeft;
    private Command enterRight;
    private Command leaveRight;
    private Boolean doFlip;

    public AutoCommands() {
        initialize();
    }

    private void initialize() {
        doFlip = true;
        if (DriverStation.getAlliance().isPresent()) {
            doFlip = DriverStation.getAlliance().get().equals(Alliance.Red);
        }
        enterLeft = createPath("enter home left");
        leaveLeft = createPath("leave home left");
        enterRight = createPath("enter home right");
        leaveRight = createPath("leave home right");
    }

    public void cancelPaths() {
        CommandScheduler.getInstance().cancel(enterLeft, enterRight, leaveLeft, leaveRight);
    }

    public void choosePath() {

        SmartDashboard.putBoolean("doFlip", doFlip);
        double x = CommandSwerveDrivetrain.getInstance().getState().Pose.getX();
        double y = CommandSwerveDrivetrain.getInstance().getState().Pose.getY();
        if (x > 4.625 && x < 12) {
            if (y > 4) {
                CommandScheduler.getInstance().schedule(enterLeft);
            } else {
                CommandScheduler.getInstance().schedule(enterRight);
            }
        } else {
            if (y > 4) {
                CommandScheduler.getInstance().schedule(leaveLeft);
            } else {
                CommandScheduler.getInstance().schedule(leaveRight);
            }
        }
    }

    private Command createPath(String pathname) {
        // Load the path we want to pathfind to and follow
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathname);
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
            return null;
        }
        if (doFlip) {
            path = path.mirrorPath();
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
