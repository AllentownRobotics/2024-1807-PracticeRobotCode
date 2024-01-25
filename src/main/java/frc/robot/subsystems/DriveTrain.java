// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

 CANSparkMax l1;
 CANSparkMax l2;
 CANSparkMax l3;
 CANSparkMax r1;
 CANSparkMax r2;
 CANSparkMax r3;

  MotorControllerGroup LeftWheels;
  MotorControllerGroup RightWheels;

  DifferentialDrive drivetrain;
  /** Creates a new DriveTrain. */
  public DriveTrain() {

    l1 = new CANSparkMax(Constants.DriveTrain.DriveTrainMotorL1, MotorType.kBrushless);
    l2 = new CANSparkMax(Constants.DriveTrain.DriveTrainMotorL2, MotorType.kBrushless);
    l3 = new CANSparkMax(Constants.DriveTrain.DriveTrainMotorL3, MotorType.kBrushless);

    r1 = new CANSparkMax(Constants.DriveTrain.DriveTrainMotorR1, MotorType.kBrushless);
    r2 = new CANSparkMax(Constants.DriveTrain.DriveTrainMotorR2, MotorType.kBrushless);
    r3 = new CANSparkMax(Constants.DriveTrain.DriveTrainMotorR3, MotorType.kBrushless);

    l1.setIdleMode(IdleMode.kCoast);
    l2.setIdleMode(IdleMode.kCoast);
    l3.setIdleMode(IdleMode.kCoast);

    r1.setIdleMode(IdleMode.kCoast);
    r2.setIdleMode(IdleMode.kCoast);
    r3.setIdleMode(IdleMode.kCoast);
  }

  public void ArcadeDrive(double moveSpeed, double rotateSpeed) {
    drivetrain.arcadeDrive(moveSpeed, rotateSpeed);
  }

  @Override
  public void periodic() {}

    // This method will be called once per scheduler run
  
}
