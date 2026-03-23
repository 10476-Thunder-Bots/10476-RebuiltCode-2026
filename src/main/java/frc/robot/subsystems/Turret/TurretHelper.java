package frc.robot.subsystems.Turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotConstants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurretHelper {
    private static TurretHelper turretHelper = null;

    public static TurretHelper getInstance() {
        if (turretHelper == null) {
            turretHelper = new TurretHelper();
        }

        return turretHelper;
    }

    private CommandSwerveDrivetrain drivetrain;

    private TurretHelper() {
        drivetrain = CommandSwerveDrivetrain.getInstance();

    }

    private Translation2d getTarget() {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            if (drivetrain.getState().Pose.getX() < RobotConstants.SwivelConstants.HUB_X) {
                return RobotConstants.SwivelConstants.HUB.minus(drivetrain.getState().Pose.getTranslation());
            }
            if (drivetrain.getState().Pose.getY() >= RobotConstants.SwivelConstants.HUB_Y) {
                return RobotConstants.SwivelConstants.UPPER_TRENCH.minus(drivetrain.getState().Pose.getTranslation());
            }
            return RobotConstants.SwivelConstants.LOWER_TRENCH.minus(drivetrain.getState().Pose.getTranslation());
        }
        if (drivetrain.getState().Pose.getX() > RobotConstants.SwivelConstants.HUB_X) {
            return RobotConstants.SwivelConstants.HUB.minus(drivetrain.getState().Pose.getTranslation());
        }
        if (drivetrain.getState().Pose.getY() >= RobotConstants.SwivelConstants.HUB_Y) {
            return RobotConstants.SwivelConstants.UPPER_TRENCH.minus(drivetrain.getState().Pose.getTranslation());
        }
        return RobotConstants.SwivelConstants.LOWER_TRENCH.minus(drivetrain.getState().Pose.getTranslation());
    }

    public double getShootVelocity() {
        // just defining some constants.
        double x = getTarget().getX();
        double y = getTarget().getY();
        double w = 1.3;
        double x2 = Math.pow(x, 2);
        double y2 = Math.pow(y, 2);
        double getDistanceFromHub = Math.sqrt(x2 + y2);
        // formula

        double top = (Math.pow(RobotConstants.SwivelConstants.GRAVITY, 2)) * (x2 + y2);
        double bottom = -((2 * (RobotConstants.SwivelConstants.GRAVITY)
                * (getDistanceFromHub) * Math.tan(RobotConstants.SwivelConstants.LAUNCH_ANGLE.in(Radians)))
                - (2 * RobotConstants.SwivelConstants.GRAVITY * w))
                * Math.pow(Math.cos(RobotConstants.SwivelConstants.LAUNCH_ANGLE.in(Radians)), 2);

        double fullFraction = top / bottom;
        return Math.sqrt(fullFraction);

    }

    public Rotation2d shootAngle() {
        double Vx = drivetrain.getChassisSpeeds().vxMetersPerSecond;
        double Vy = drivetrain.getChassisSpeeds().vyMetersPerSecond;
        double rho = Math.atan2(getTarget().getY(), getTarget().getX());
        double Vix = getShootVelocity() * Math.cos(rho) - Vx;
        double Viy = getShootVelocity() * Math.sin(rho) - Vy;
        Rotation2d thetaI = new Rotation2d(-Math.atan2(Viy, Vix));
        return thetaI.plus(drivetrain.getState().Pose.getRotation());
    }

    public AngularVelocity launchSpeed() {
        Distance distanceFromTarget = Meters
                .of(Math.sqrt(Math.pow(getTarget().getX(), 2) + Math.pow(getTarget().getY(), 2)));
        double[] distances = { 24, 36, 52, 56, 75, 78, 89, 107 };
        double[] speeds = { 2500, 2700, 3200, 3300, 3750, 3900, 4100, 4250 };
        double hubRobotOffset = 35
        for i = 0; i< distances.length;i++
    }

}