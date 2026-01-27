package frc.robot.generated;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;

public final class RobotConstants {
    public static class Autos {
        public static final PathPlannerAuto Left = new PathPlannerAuto("Left");
        public static final PathPlannerAuto Right = new PathPlannerAuto("Right");
        public static final PathPlannerAuto Middle = new PathPlannerAuto("Middle");

        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(0, 0, 0);
    }
}
