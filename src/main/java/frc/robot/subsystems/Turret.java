package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    PIDController turretController;
    AnalogEncoder analogEncoder;
    Dashboard dashboard;
    TalonFX turretMotor;
    public Turret(Dashboard dashboard){
        turretController = new PIDController(2, 0, 0);
        turretController.disableContinuousInput();
        analogEncoder = new AnalogEncoder(0,10.0,0);
        this.dashboard = dashboard;
        turretMotor = new TalonFX(0);
        turretMotor.getConfigurator().apply(new TalonFXConfiguration());
        
    }
    public Command setTurretSetpoint(){
        return Commands.run(()->turretController.setSetpoint(dashboard.getTurretSetpoint().in(Radians)));
    }
    @Override
    public void periodic() {
        if(!turretController.atSetpoint())
        {
            turretMotor.setVoltage(turretController.calculate(analogEncoder.get(),dashboard.getTurretSetpoint().in(Radians)));
        }
    }
}
