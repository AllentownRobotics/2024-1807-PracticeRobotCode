// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {

  double desiredAngle;
  CANSparkMax wrist;
  SparkPIDController pid;
  

  /** Creates a new Wrist. */
  public Wrist() {

    wrist = new CANSparkMax(0, MotorType.kBrushless);

    wrist.restoreFactoryDefaults();

    wrist.setIdleMode(IdleMode.kBrake);
    wrist.getAbsoluteEncoder(Type.kDutyCycle);
    
    pid = wrist.getPIDController();

    pid.setP(WristConstants.wristP);
    pid.setI(WristConstants.wristI);
    pid.setD(WristConstants.wristD);
    
    wrist.burnFlash();

    desiredAngle = 0;
  }

  @Override
  public void periodic() {

    wrist.getPIDController().setReference(desiredAngle, ControlType.kPosition);

  }

  public void moveToSetpoint(double desiredAngle) {

    this.desiredAngle = desiredAngle;
    
  }
}
