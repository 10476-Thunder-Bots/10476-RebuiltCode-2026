package frc.robot.subsystems;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.RobotConstants;
/* 
public class Dashboard extends SubsystemBase { 
    private final SendableChooser<String> autoStartPositionChooser;
    private PathPlannerAuto selectAuto;

    public Dashboard(){
        autoStartPositionChooser = new SendableChooser<String>();
        autoStartPositionChooser.addOption("Left", "Left");
        autoStartPositionChooser.addOption("Middle", "Middle");
        autoStartPositionChooser.addOption("Right", "Right");
        SmartDashboard.putData("StartPosition", autoStartPositionChooser);
    }


    @Override
    public void periodic() {
     switch (autoStartPositionChooser.getSelected()) {
        case "Left":
            selectAuto = RobotConstants.Autos.Left;
            break;
        case "Middle":
            selectAuto = RobotConstants.Autos.Middle;
            break;
        case "Right":
            selectAuto = RobotConstants.Autos.Right;
            break;
        default:

            break;
     }  

     if (selectAuto != null){

     }

        
    }
    public Command getSelectAuto(){
        return selectAuto;
    }
}
*/