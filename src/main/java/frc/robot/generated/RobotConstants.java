package frc.robot.generated;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;


public final class RobotConstants {
    public static class Autos {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(0, 0, 0);
    }

    public static class LimeLight {
        public static final String LEFT_LIMELIGHT_NAME = "limelight-uncleft";
        public static final String RIGHT_LIMELIGHT_NAME = "limelight-unleft";
        private static final double XY_STD_DEV = 0.2;
        private static final double THEATA_STD_DEV = 1.5;
        public static final Matrix<N3, N1> STD_DEVS = VecBuilder.fill(XY_STD_DEV, XY_STD_DEV, THEATA_STD_DEV);
    }
    public static class Turret {
        @SuppressWarnings("unlikely-arg-type")
        public static final double HUB_X = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? 4.625 : 12;
        public static final double HUB_Y = 4;
        public static final int ENCODER_ID = 0;
        public static final int TURRET_CAN_ID = 27;
        public static final double TURRET_KP = 15;
        public static final double TURRET_KI = 0;
        public static final double TURRET_KD = 0;
        public static final AngularVelocity TURRET_MAX_VEL = DegreesPerSecond.of(180);
        public static final AngularAcceleration TURRET_MAX_ACC = DegreesPerSecondPerSecond.of(180);
    }
}
