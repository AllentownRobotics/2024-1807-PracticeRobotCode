// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmCommand extends CommandBase {

  int commandValue;
  /** Creates a new ArmCommand. */
  public ArmCommand(int commandValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (commandValue) {
      
      case ArmConstants.kArmToggleCommandIndex: 
      RobotContainer.arm.ToggleArm();

      case ArmConstants.kArmForwardCommandIndex: 
      RobotContainer.arm.ForwardArm();

      case ArmConstants.kArmReverseCommandIndex: 
      RobotContainer.arm.ReverseArm();
      
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
