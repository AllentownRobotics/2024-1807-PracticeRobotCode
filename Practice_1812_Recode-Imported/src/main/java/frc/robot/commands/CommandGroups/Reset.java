// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ArmCommands.ArmSetCommand;
import frc.robot.commands.ClawCommands.ClawSetCommand;
import frc.robot.commands.WristCommands.WristToSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Reset extends SequentialCommandGroup {

  /** Creates a new Reset. */
  public Reset(Claw claw, Arm arm, Wrist wrist) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ClawSetCommand(claw, Value.kForward), 

    new ParallelCommandGroup
    (new WristToSetpoint(wrist, WristConstants.wristRestSetpoint), 

    new ArmSetCommand(arm, Value.kReverse))
    
    );
  }
}
