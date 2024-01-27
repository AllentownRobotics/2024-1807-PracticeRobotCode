// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class AutoCollect extends Command {

  Claw claw;

  /** Creates a new CloseIfInRange. */
  public AutoCollect(Claw claw) {

    this.claw = claw;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new ClawClosed(claw);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (claw.distance.getRange() < 4) {

      return true;

    } else {

      return false;
    }
  }
}
