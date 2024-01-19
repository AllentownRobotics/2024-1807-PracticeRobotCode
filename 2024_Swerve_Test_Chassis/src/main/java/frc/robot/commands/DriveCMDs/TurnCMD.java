// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class TurnCMD extends Command {

  private DriveTrain drive;
  private Limelight limelight;
  
  
  private PIDController pidController;
  public TurnCMD(DriveTrain drive, Limelight limelight) {
      this.drive = drive;
      this.limelight = limelight;

      pidController = new PIDController(.01, 0, 0);
      addRequirements(drive);
  }

  @Override
  public void execute() {
      drive.drive(
          0,
          0,
          pidController.calculate(limelight.getX()),
          true, true);
  }
}
