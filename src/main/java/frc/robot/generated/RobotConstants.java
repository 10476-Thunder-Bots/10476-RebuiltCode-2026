package frc.robot.generated;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class RobotConstants {
    public static class Autos {
        public static final PathPlannerAuto Left = new PathPlannerAuto("Left");
        public static final PathPlannerAuto Right = new PathPlannerAuto("Right");
        public static final PathPlannerAuto Middle = new PathPlannerAuto("Middle");

        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(0, 0, 0);
    }
        public final class LimeLight{
        public static final String LEFT_LIMELIGHT_NAME = "limelight-uncleft";
        public static final String RIGHT_LIMELIGHT_NAME = "limelight-uncleright";
        private static final double XY_STD_DEV = 0.2;
        private static final double THEATA_STD_DEV = 1.5;
        public static final Matrix<N3,N1> STD_DEVS = VecBuilder.fill(XY_STD_DEV,XY_STD_DEV,THEATA_STD_DEV);
    }
}
