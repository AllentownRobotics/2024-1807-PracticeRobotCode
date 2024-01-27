// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  DoubleSolenoid arm;

  /** Creates a new Arm. */
  public Arm() {
    arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void toggleArm() {

    if (arm.get().equals(Value.kForward)) {

      arm.set(Value.kReverse);

    } else {

      arm.set(Value.kForward);
    }
  }

  public void setArm(Value value) {
    arm.set(value);
  }
}
