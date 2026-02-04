package frc.robot.subsystems;

import com.pathplanner.lib.commands.PathPlannerAuto;

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
        
     }  

        
}

