package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.RobotConstants;

public class Dashboard extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private Turret turret;

    public Dashboard(CommandSwerveDrivetrain drivetrain, Turret turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Angle", turret.getTurretSetpoint().in(Degrees));
    }

       public double getShootVelocity(){
        // just defining some constants.
        double x = RobotConstants.Turret.HUB_X - drivetrain.getState().Pose.getX();
        double y = RobotConstants.Turret.HUB_Y - drivetrain.getState().Pose.getY();
        double w = 1.3;
        double theta = Math.toRadians(85);
        double x2 = Math.pow(x, 2);
        double y2 = Math.pow(y, 2);
        double g = -9.8;
        double getDistanceFromHub = Math.sqrt(x2 + y2);
        // formula
        
        double top = (Math.pow(g,2))*(x2 + y2);
        double bottom = -((2*(g)*(getDistanceFromHub)*Math.tan(theta))-(2*g*w)) * Math.pow(Math.cos(theta), 2);
        double fullFraction = top/bottom;
        return Math.sqrt(fullFraction);
    }

        public double shootAngle(){
            double x = RobotConstants.Turret.HUB_X - drivetrain.getState().Pose.getX();
            double y = RobotConstants.Turret.HUB_Y - drivetrain.getState().Pose.getY();
            double Vx = drivetrain.getChassisSpeeds().vxMetersPerSecond;
            double Vy = drivetrain.getChassisSpeeds().vyMetersPerSecond;
            double rho = Math.atan(y/x);
            double Vix = getShootVelocity() * Math.cos(rho) - Vx;
            double Viy = getShootVelocity() * Math.sin(rho) - Vy;
            double thetaI = Math.toDegrees(Math.atan(Viy/Vix));
            return thetaI;

        }

}
