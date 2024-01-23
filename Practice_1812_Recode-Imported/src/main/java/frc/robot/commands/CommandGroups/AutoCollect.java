// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ArmCommands.ArmSetCommand;
import frc.robot.commands.ClawCommands.ClawSetCommand;
import frc.robot.commands.ClawCommands.CloseIfInRange;
import frc.robot.commands.WristCommands.WristToSetpoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCollect extends SequentialCommandGroup {

  /** Creates a new AutoCollect. */
  public AutoCollect(Arm arm, Claw claw, Wrist wrist, Ultrasonic distance) {

    Value value = Value.kForward;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ArmSetCommand(arm, Value.kReverse),

    new ClawSetCommand(claw, Value.kReverse), 

    new WristToSetpoint(wrist, WristConstants.wristLowSetpoint),

    new CloseIfInRange(distance, claw, value));
  }
}
