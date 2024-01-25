// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Claw;

public class ClawClosed extends CommandBase {
 
 Claw cClaw;
 CommandXboxController cController;

  /** Creates a new ClawClosed. */
  public ClawClosed(Claw cClaw) {
   this.cClaw = cClaw;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cClaw.Thomasisevil(Value.kForward);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
