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

}
