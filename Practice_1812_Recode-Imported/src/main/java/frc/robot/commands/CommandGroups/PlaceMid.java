// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ArmUp;
import frc.robot.commands.ClawCommands.ClawOpen;
import frc.robot.commands.WristCommands.WristMid;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceMid extends SequentialCommandGroup {

  /** Creates a new PlaceHigh. */
  public PlaceMid(Arm arm, Claw claw, Wrist wrist) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ArmUp(arm),

    new WristMid(wrist),

    new ClawOpen(claw));
  }
}
