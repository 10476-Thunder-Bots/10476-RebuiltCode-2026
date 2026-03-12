package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret.Shooter;
import frc.robot.subsystems.Turret.Swivel;
import frc.robot.subsystems.Turret.TurretHelper;

public class Dashboard extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Swivel swivel;
    private TurretHelper turretHelper;
    private Intake intake;

    private static Dashboard dashboard = null;

    public static Dashboard getInstance() {
        if (dashboard == null) {
            dashboard = new Dashboard();
        }
        return dashboard;
    }

    private Dashboard() {
        intake = Intake.getInstance();
        turretHelper = TurretHelper.getInstance();
        shooter = Shooter.getInstance();
        swivel = Swivel.getInstance();
        drivetrain = CommandSwerveDrivetrain.getInstance();

        SmartDashboard.putNumber("Set Swivel Angle", swivel.manuelSwivelAngle().in(Degrees));

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Positition of Intake", intake.getIntakePosition().in(Degrees));
        SmartDashboard.putNumber("MPS of shooter", shooter.getLinearVelocity().in(MetersPerSecond));
        SmartDashboard.putNumber("RPM of shooter", shooter.getAngularVelocity().in(RPM));
        SmartDashboard.putNumber("Swivel angle", swivel.getAngle().in(Degrees));
        SmartDashboard.putNumber("Shoot Velocity", turretHelper.getShootVelocity());
        SmartDashboard.putNumber("Set Speed", manuelShootSpeed());
    }

    public double manuelShootSpeed() {
        return SmartDashboard.getNumber("Set Speed", turretHelper.getShootVelocity());
    }

}
