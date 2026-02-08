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

    public Dashboard(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Angle", getTurretSetpoint().in(Degrees));
     }  
    public Angle getTurretSetpoint(){
        Translation2d toTarget = RobotConstants.Turret.HUB.minus(drivetrain.getState().Pose.getTranslation());
        Rotation2d aimAngle = new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
        return aimAngle.minus(drivetrain.getPigeon2().getRotation2d()).getMeasure();
    }
    public double getDistanceFromHub(){
        return Math.sqrt(Math.pow(drivetrain.getState().Pose.getY()-RobotConstants.Turret.HUB_Y, 2)+Math.pow(drivetrain.getState().Pose.getX()-RobotConstants.Turret.HUB_X,2));
    }
}

