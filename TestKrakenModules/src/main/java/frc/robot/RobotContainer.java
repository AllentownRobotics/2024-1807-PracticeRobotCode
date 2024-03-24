// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveCMD;
import frc.robot.commands.MoveVoltsCMD;
import frc.robot.commands.MoveVoltsConstantCMD;
import frc.robot.commands.TestControlCMD;
import frc.robot.commands.TestOpenLoopControlCMD;
import frc.robot.subsystems.KrakenSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final KrakenSubsystem krakenSubsystem = new KrakenSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    krakenSubsystem.setDefaultCommand(new MoveCMD(krakenSubsystem, m_driverController));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driverController.a().whileTrue(new TestControlCMD(krakenSubsystem, m_driverController));
    m_driverController.b().whileTrue(new TestOpenLoopControlCMD(krakenSubsystem, m_driverController));
    
    m_driverController
        .povUp()
        .and(m_driverController.rightBumper())
        .whileTrue(krakenSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController
        .povRight()
        .and(m_driverController.rightBumper())
        .whileTrue(krakenSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController
        .povDown()
        .and(m_driverController.rightBumper())
        .whileTrue(krakenSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController
        .povLeft()
        .and(m_driverController.rightBumper())
        .whileTrue(krakenSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
