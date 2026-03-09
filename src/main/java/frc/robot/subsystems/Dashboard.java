package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Turret.Shooter;
import frc.robot.subsystems.Turret.Swivel;

public class Dashboard extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    Shooter shooter = Shooter.getInstance();
    Swivel swivel = Swivel.getInstance();
    Double intialAngle = swivel.getAnalogPotentiometer();

    private static Dashboard dashboard = null;

    public static Dashboard getInstance() {
        if (dashboard == null) {
            dashboard = new Dashboard();
        }
        return dashboard;
    }

    private Dashboard() {
        drivetrain = CommandSwerveDrivetrain.getInstance();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoot Vel", getShootVelocity());
        SmartDashboard.putNumber("Set Speed", changeShootSpeed());
        SmartDashboard.putNumber("Set Swivel Angle", changeSwivelAngle());
    }

    public double getShootVelocity() {
        // just defining some constants.
        double x = RobotConstants.SwivelConstants.HUB_X - drivetrain.getState().Pose.getX();
        double y = RobotConstants.SwivelConstants.HUB_Y - drivetrain.getState().Pose.getY();
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
        Rotation2d thetaI = new Rotation2d(Math.atan2(Viy, Vix));
        return thetaI.minus(new Rotation2d(90));

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

    public AngularVelocity linearInterpolation() {
        // this is just a template and serves no current use
        double y1 = 0;
        double y2 = 0;
        double x1 = 0;
        double x2 = 0;
        double y = getShootVelocity();

        double m = (y2 - y1) / (x2 - x1);

        double x = ((y - y1) / m) + x1;

        return AngularVelocity.ofBaseUnits(x, RPM);

    }

    public double changeShootSpeed(){
        return SmartDashboard.getNumber("Set Speed", getShootVelocity());
    }

        public double changeSwivelAngle(){
        double i = SmartDashboard.getNumber("Set Swivel Angle", shootAngle().getDegrees());
        Angle j = Rotation2d.fromDegrees(i).minus(Rotation2d.fromDegrees(intialAngle)).minus(drivetrain.getRotation3d().toRotation2d()).getMeasure();
        return j.in(Degrees);
    }

}
