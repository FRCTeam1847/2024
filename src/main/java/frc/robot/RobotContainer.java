// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DropCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        public static CommandXboxController m_driverController = new CommandXboxController(
                        OperatorConstants.kDriverControllerPort);

        private final SendableChooser<Command> m_chooser = new SendableChooser<>();

        // The robot's subsystems
        private final LauncherSubsystem launchSubsystem;
        private final LightsSubsystem lightSubSystem;
        private final DriveTrainSubsystem driveSyubsystem;
        private final ClimberSubsystem climberSubsystem;

        // The robots commands
        private final IntakeCommand intakeCommand;
        private final ShootCommand shootCommand;
        private final DropCommand dropCommand;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                launchSubsystem = new LauncherSubsystem();
                lightSubSystem = new LightsSubsystem();
                driveSyubsystem = new DriveTrainSubsystem();
                climberSubsystem = new ClimberSubsystem();

                intakeCommand = new IntakeCommand(launchSubsystem, lightSubSystem);
                shootCommand = new ShootCommand(launchSubsystem, lightSubSystem);
                dropCommand = new DropCommand(launchSubsystem, lightSubSystem);

                m_chooser.setDefaultOption("Drive Back", Autos.DriveBackwardsInches(driveSyubsystem, 24));
                m_chooser.setDefaultOption("Rotate", Autos.Rotate(driveSyubsystem, 30));
                m_chooser.addOption("Drive Back + Turn", Autos.DriveInchesRotate(driveSyubsystem, 24, -30));
                m_chooser.addOption("Shoot and drive back", Autos.ShootRotateDriveBackwards(driveSyubsystem, launchSubsystem, lightSubSystem));
                SmartDashboard.putData("Auto choices", m_chooser);

                // Configure the trigger bindings
                configureBindings();
        }

        private void configureBindings() {
                // // Intake logic
                new Trigger(launchSubsystem.hasTopNote)
                                .onTrue(intakeCommand);

                //  m_driverController.leftBumper().toggleOnTrue(intakeCommand);

                // Launcher
                m_driverController
                                .x()
                                .toggleOnTrue(shootCommand);
                m_driverController
                                .y()
                                .toggleOnTrue(dropCommand);
                // Climber
                m_driverController.a().whileTrue(climberSubsystem.Climb());
                m_driverController.b().whileTrue(climberSubsystem.Lower());
                // Driver
                driveSyubsystem.setDefaultCommand(
                                driveSyubsystem.arcadeDriveCommand(
                                                () -> -m_driverController.getLeftY(),
                                                () -> -m_driverController.getLeftX()));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return m_chooser.getSelected();
        }
}
