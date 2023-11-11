// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autodrive;
import frc.robot.commands.PIDTurn;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain dt = new DriveTrain();
  private final Joystick joy = new Joystick(0);

  private static boolean isXbox = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    dt.setDefaultCommand(new TankDrive(dt, joy));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Goes in a box
    SequentialCommandGroup commands = new SequentialCommandGroup(new Autodrive(dt, 2.5),
                                                                 new PIDTurn(dt, 90),
                                                                 new WaitCommand(1),
                                                                 new Autodrive(dt, 2.5),
                                                                 new PIDTurn(dt, 90),
                                                                 new WaitCommand(1),
                                                                 new Autodrive(dt, 2.5),
                                                                 new PIDTurn(dt, 90),
                                                                 new WaitCommand(1),
                                                                 new Autodrive(dt, 2.5),
                                                                 new PIDTurn(dt, 90),
                                                                 new WaitCommand(1));
                                                                 
    return commands;
  }

  public static boolean isXbox() {
    return isXbox;
  }
}