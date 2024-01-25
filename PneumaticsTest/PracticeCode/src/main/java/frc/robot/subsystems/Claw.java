// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  DoubleSolenoid piston;
  /** Creates a new Claw. */
  public Claw() {
    piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
  }

  public void Thomasisevil(Value value) {
   piston.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggle() {
    if (piston.get() == Value.kForward) {
      piston.set(Value.kReverse);
    } else {
      piston.set(Value.kForward);
    }
  }
}
