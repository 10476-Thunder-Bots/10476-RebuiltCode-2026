package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.commands.PathPlannerAuto;

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
        SmartDashboard.putNumber("Turret Angle", getTurretSetpoint().in(Radians));
     }  
     public Angle getTurretSetpoint(){
        return Angle.ofBaseUnits(Math.atan2(drivetrain.getState().Pose.getY()-4.0, drivetrain.getState().Pose.getX()-4.622) 
        - drivetrain.getState().Pose.getRotation().getRadians(), Radians);
    }
    public double getDistanceFromHub(){
        return Math.sqrt(Math.pow(drivetrain.getState().Pose.getY()-5, 2)+Math.pow(drivetrain.getState().Pose.getX(), -5));
    }
        
}

