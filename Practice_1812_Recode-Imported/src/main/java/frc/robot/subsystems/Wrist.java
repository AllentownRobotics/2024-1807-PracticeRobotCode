// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

  double desiredAngle;
  CANSparkMax wrist;
  

  /** Creates a new Wrist. */
  public Wrist() {

    wrist = new CANSparkMax(0, MotorType.kBrushless);

  }

  @Override
  public void periodic() {

    wrist.getPIDController().setReference(desiredAngle, ControlType.kPosition);

  }

  public void moveToSetpoint(double desiredAngle) {

    this.desiredAngle = desiredAngle;
    
  }
}
