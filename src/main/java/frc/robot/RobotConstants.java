package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class RobotConstants {
    public static class Autos {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(0, 0, 0);
    }

    public static class LimeLight {
        public static final String LEFT_LIMELIGHT_NAME = "limelight-uncleft";
        public static final String RIGHT_LIMELIGHT_NAME = "limelight-unleft";
        private static final double XY_STD_DEV = 0.2;
        private static final double THETA_STD_DEV = 9999999;
        public static final Matrix<N3, N1> STD_DEVS = VecBuilder.fill(XY_STD_DEV, XY_STD_DEV, THETA_STD_DEV);
    }

    public static class SwivelConstants {
        private static final boolean allianceCheck = DriverStation.getAlliance().isPresent();
        private static final Alliance ALLIANCE = allianceCheck ? DriverStation.getAlliance().get()
                : DriverStation.Alliance.Red;
        public static final double HUB_X = ALLIANCE.equals(DriverStation.Alliance.Blue) ? 4.625
                : 12;
        public static final double HUB_Y = 4;
        public static final double TRENCH_X = ALLIANCE.equals(DriverStation.Alliance.Blue) ? 2
                : 14;
        public static final double UPPER_TRENCH_Y = 6;
        public static final double LOWER_TRENCH_Y = 1.5;
        public static final Translation2d HUB = new Translation2d(HUB_X, HUB_Y);
        public static final Translation2d UPPER_TRENCH = new Translation2d(TRENCH_X, UPPER_TRENCH_Y);
        public static final Translation2d LOWER_TRENCH = new Translation2d(TRENCH_X, LOWER_TRENCH_Y);
        public static final int ENCODER_ID = 0;
        public static final int SWIVEL_CAN_ID = 14;
        public static final double SWIVEL_KP = 53;
        public static final double SWIVEL_KI = 0;
        public static final double SWIVEL_KD = 0;
        public static final AngularVelocity SWIVEL_MAX_VEL = DegreesPerSecond.of(590);
        public static final AngularAcceleration SWIVEL_MAX_ACC = DegreesPerSecondPerSecond.of(46000);
        public static final double GRAVITY = -9.81;
        public static final Angle LAUNCH_ANGLE = Degrees.of(77);
    }

    public static class ShooterConstants {
        public static final int SHOOTER_CAN_ID = 16;
        public static final int FOLLOWER_CAN_ID = 15;
        public static final double SHOOTER_KP = 0;
        public static final double SHOOTER_KV = 0.10695;
    }

    public static class IntakeConstants {
        public static final double Intake_KP = 1;
        public static final double Intake_KD = 0;
        public static final double Intake_KI = 0;
    }
}
