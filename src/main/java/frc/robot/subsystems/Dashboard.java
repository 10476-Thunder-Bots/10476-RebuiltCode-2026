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
import frc.robot.subsystems.Turret.TurretHelper;

public class Dashboard extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter = Shooter.getInstance();
    private Swivel swivel = Swivel.getInstance();
    private TurretHelper turretHelper = TurretHelper.getInstance();
    double intialAngle;

    private static Dashboard dashboard = null;

    public static Dashboard getInstance() {
        if (dashboard == null) {
            dashboard = new Dashboard();
        }
        return dashboard;
    }

    private Dashboard() {
        drivetrain = CommandSwerveDrivetrain.getInstance();
        intialAngle = swivel.getAnalogPotentiometer();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoot Vel", turretHelper.getShootVelocity());
        SmartDashboard.putNumber("Set Speed", changeShootSpeed());
        SmartDashboard.putNumber("Set Swivel Angle", changeSwivelAngle().baseUnitMagnitude());
        SmartDashboard.setDefaultNumber("Set Swivel Angle", 0);
    }

   

    public double changeShootSpeed(){
        return SmartDashboard.getNumber("Set Speed", turretHelper.getShootVelocity());
    }

        public Angle changeSwivelAngle(){
        double setAngle = SmartDashboard.getNumber("Set Swivel Angle",0);
        return Rotation2d.fromDegrees(setAngle).minus(Rotation2d.fromDegrees(intialAngle)).minus(drivetrain.getRotation3d().toRotation2d()).getMeasure();
        
    }

}
