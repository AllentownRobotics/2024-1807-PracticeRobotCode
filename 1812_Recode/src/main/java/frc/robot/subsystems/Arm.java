// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  DoubleSolenoid armLeft;
  DoubleSolenoid armRight;

  CANSparkMax wrist;

  /** Creates a new Arm. */
  public Arm() {

    armLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    armRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);

    wrist = new CANSparkMax(0, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void ToggleArm() {
    armLeft.toggle();
    armRight.toggle();
  }

  public void ForwardArm() {
    armLeft.set(Value.kForward);
    armRight.set(Value.kForward);
  }

  public void ReverseArm() {
    armLeft.set(Value.kReverse);
    armRight.set(Value.kReverse);
  }
}
