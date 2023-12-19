// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  DoubleSolenoid claw;
  DoubleSolenoid wrist;
  
  /** Creates a new Claw. */
  public Claw() {
    claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    wrist = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ToggleClaw() {
    claw.toggle();
  }

  public void CloseClaw() {
    claw.set(Value.kForward);
  }

  public void OpenClaw() {
    claw.set(Value.kReverse);
  }

  public void ToggleWrist() {
    wrist.toggle();
  }

  public void WristUp() {
    wrist.set(Value.kForward);
  }
  
  public void WristDown() {
    wrist.set(Value.kReverse);
  }
}
