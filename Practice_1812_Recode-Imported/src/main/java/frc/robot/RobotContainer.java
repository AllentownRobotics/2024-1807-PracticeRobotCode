// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CompressCommand;
import frc.robot.commands.ArmCommands.ArmToggleCommand;
import frc.robot.commands.ClawCommands.ClawToggleCommand;
import frc.robot.commands.ClawCommands.AutoCollect;
import frc.robot.commands.CommandGroups.PlaceHigh;
import frc.robot.commands.CommandGroups.PlaceLow;
import frc.robot.commands.CommandGroups.PlaceMid;
import frc.robot.commands.CommandGroups.Reset;
import frc.robot.commands.WristCommands.WristDefault;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Compress;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;
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
  public final static Arm arm = new Arm();
  public final static Claw claw = new Claw();
  public final static Wrist wrist = new Wrist();
  public final static Compress compressor = new Compress();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    compressor.setDefaultCommand(new CompressCommand(compressor));
    
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

    m_operatorController.y().onTrue(new ArmToggleCommand(arm));

    m_operatorController.x().onTrue(new ClawToggleCommand(claw));

    m_operatorController.b().onTrue(new WristDefault(wrist));

    m_operatorController.rightBumper().onTrue(new Reset(claw, arm, wrist));

    m_operatorController.leftBumper().whileTrue(new AutoCollect(claw));

    m_operatorController.povDown().onTrue(new PlaceLow(arm, claw, wrist));

    m_operatorController.povLeft().onTrue(new PlaceMid(arm, claw, wrist));

    m_operatorController.povRight().onTrue(new PlaceMid(arm, claw, wrist));

    m_operatorController.povUp().onTrue(new PlaceHigh(arm, claw, wrist));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
