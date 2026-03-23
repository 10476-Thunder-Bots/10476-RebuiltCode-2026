// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret.Loader;
import frc.robot.subsystems.Turret.Shooter;
import frc.robot.subsystems.Turret.Swivel;
import frc.robot.subsystems.Turret.TurretHelper;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandJoystick joystick = new CommandJoystick(0);

        private final CommandXboxController xboxjoystick = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

        private final Loader loader = Loader.getInstance();

        public final Dashboard dashboard = Dashboard.getInstance();

        public final Swivel swivel = Swivel.getInstance();

        public final TurretHelper turretHelper = TurretHelper.getInstance();

        public final Shooter shooter = Shooter.getInstance();

        public final Intake intake = Intake.getInstance();

        private final AutoCommands autoCommands = new AutoCommands();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                configureBindings();
                NamedCommands.registerCommand("pathfind", drivetrain.runOnce(() -> autoCommands.choosePath()));
                NamedCommands.registerCommand("shoot", CompositeCommands.swivelShoot());
                NamedCommands.registerCommand("run loader", CompositeCommands.runLoader());
                NamedCommands.registerCommand("run intake", CompositeCommands.intakeOn());
                NamedCommands.registerCommand("retract intake", CompositeCommands.intakeOff());
                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("autoChooser", autoChooser);
        }

        private void configureBindings() {

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getY() * MaxSpeed) // Drive
                                                                                                               // forward
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // Y
                                                                                                               // (forward)
                                                .withVelocityY(-joystick.getX() * MaxSpeed) // Drive left with negative
                                                                                            // X (left)
                                                .withRotationalRate(
                                                                Math.copySign(Math.pow(-joystick.getZ(), 2),
                                                                                -joystick.getZ()) * MaxAngularRate) // Drive
                                                                                                                    // counterclockwise
                                                                                                                    // with
                                                                                                                    // negative
                                                                                                                    // X
                                                                                                                    // (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                shooter.setDefaultCommand(shooter.set(0));
                swivel.setDefaultCommand(swivel.setdutyCycle(0));
                loader.setDefaultCommand(loader.run(() -> loader.setLoader(0)));
                joystick.button(7).whileTrue(drivetrain.applyRequest(() -> brake));

                // reset the field-centric heading on button 6 press
                joystick.button(6).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


                joystick.button(5)
                                .whileTrue(shooter.run(() -> shooter.setVelocity(
                                                RPM.of(dashboard.manuelShootSpeed()))));

                joystick.button(4).whileTrue(CompositeCommands.shakeBot());
                joystick.button(3).toggleOnTrue(drivetrain.runOnce(() -> autoCommands.choosePath()));
                joystick.button(2).toggleOnTrue(drivetrain.runOnce(() -> autoCommands.cancelPaths()));
                xboxjoystick.button(3).whileTrue(CompositeCommands.runLoader());
                xboxjoystick.button(4).toggleOnTrue(CompositeCommands.swivelShoot());
                //xboxjoystick.button(2).toggleOnTrue(CompositeCommands.intakeOn())
                               // .onFalse(CompositeCommands.intakeOff());
                xboxjoystick.button(2).whileTrue(intake.run(()-> intake.pushIntakeOut()));
                drivetrain.registerTelemetry(logger::telemeterize);

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
