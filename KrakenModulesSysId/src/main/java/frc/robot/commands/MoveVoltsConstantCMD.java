// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.KrakenSubsystem;

public class MoveVoltsConstantCMD extends Command {
  KrakenSubsystem krakenSubsystem;
  CommandXboxController controller;
  /** Creates a new MoveCMD. */
  public MoveVoltsConstantCMD(KrakenSubsystem krakenSubsystem, CommandXboxController controller) {
    this.krakenSubsystem = krakenSubsystem;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(krakenSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    krakenSubsystem.setVoltage(krakenSubsystem.getConstantVoltage());
    SmartDashboard.putNumber("commanded constant voltage", krakenSubsystem.getConstantVoltage());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    krakenSubsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
