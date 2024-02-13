// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeAnimationCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeFeedCommand;
import frc.robot.commands.NoteCollectedAnimationCommand;
import frc.robot.commands.NoteMissingCommand;
import frc.robot.commands.ShootingAnimationCommand;
import frc.robot.commands.ShootingCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  // The robot's subsystems
  private final LightsSubsystem lightSubSystem = new LightsSubsystem();
  private final ShootingSubsystem shootingSubsystem = new ShootingSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();

  // The robots commands
  private final ShootingCommand shootingCommand = new ShootingCommand(shootingSubsystem);
  private final ShootingAnimationCommand shootingAnimation = new ShootingAnimationCommand(lightSubSystem);
  private final IntakeAnimationCommand intakeAnimationCommand = new IntakeAnimationCommand(lightSubSystem);
  private final NoteCollectedAnimationCommand noteCollectedAnimationCommand = new NoteCollectedAnimationCommand(
      lightSubSystem);
  private final NoteMissingCommand noteMissingCommand = new NoteMissingCommand(lightSubSystem);
  private final IntakeCommand intakeCommand = new IntakeCommand(shootingSubsystem);
  private final FeedCommand feedCommand = new FeedCommand(feederSubsystem);
  private final IntakeFeedCommand intakeFeedCommand = new IntakeFeedCommand(feederSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // new
    // Trigger(shootingSubsystem.NoteSwitchPressed()).onTrue(noteCollectedAnimationCommand);
    // new
    // Trigger(shootingSubsystem.NoteSwitchNotPressed()).onTrue(noteMissingCommand);
   
    new Trigger(shootingSubsystem.TopSwitchPressed()).onTrue(
       new ParallelCommandGroup(
            intakeAnimationCommand,
            intakeCommand,
            intakeFeedCommand)
            .onlyIf(shootingSubsystem.NoteSwitchNotPressed())
            .until(shootingSubsystem.NoteSwitchPressed())
            .andThen(noteCollectedAnimationCommand)
    );
    // m_driverController.b().toggleOnTrue(
    //     new ParallelCommandGroup(
    //         intakeAnimationCommand,
    //         intakeCommand,
    //         intakeFeedCommand)
    //         .onlyIf(shootingSubsystem.NoteSwitchNotPressed())
    //         .until(shootingSubsystem.NoteSwitchPressed())
    //         .andThen(noteCollectedAnimationCommand));

    m_driverController.x().toggleOnTrue(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                shootingCommand.withTimeout(1)
                    .asProxy()
                    .andThen(feedCommand)
                    // .onlyIf(shootingSubsystem.NoteSwitchPressed())
                    .until(shootingSubsystem.NoteSwitchNotPressed()),
                new WaitCommand(2),
                new ParallelCommandGroup(
                    shootingSubsystem.StopCommand(),
                    feederSubsystem.StopCommand())),
            shootingAnimation.onlyIf(shootingSubsystem.NoteSwitchPressed())
                .until(shootingSubsystem.NoteSwitchNotPressed())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous
  // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
