// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClawConstants;

public class ClawCommand extends CommandBase {
  int commandValue;
  /** Creates a new ClawCommand. */
  public ClawCommand(int commandValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (commandValue) {
      
      case ClawConstants.kClawToggleCommandIndex: 
      RobotContainer.claw.ToggleClaw();

      case ClawConstants.kClawCloseCommandIndex: 
      RobotContainer.claw.CloseClaw();

      case ClawConstants.kClawOpenCommandIndex: 
      RobotContainer.claw.OpenClaw();

      case ClawConstants.kWristToggleCommandIndex: 
      RobotContainer.claw.ToggleWrist();

      case ClawConstants.kWristDownCommandIndex: 
      RobotContainer.claw.WristDown();

      case ClawConstants.kWristUpCommandIndex: 
      RobotContainer.claw.WristUp();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
