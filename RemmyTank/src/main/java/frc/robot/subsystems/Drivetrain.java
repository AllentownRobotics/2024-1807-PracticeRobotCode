// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  CANSparkMax l1;
  CANSparkMax r1;
  CANSparkMax l2;
  CANSparkMax r2;

  MotorControllerGroup leftWheels;
  MotorControllerGroup rightWheels;

  DifferentialDrive drivetrain;

  CommandXboxController m_driverController;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    // defines motors and motor controller groups

    l1 = new CANSparkMax(Constants.DrivetrainConstants.kDrivetrainMotorL1, MotorType.kBrushless);

    l2 = new CANSparkMax(Constants.DrivetrainConstants.kDrivetrainMotorL2, MotorType.kBrushless);

    r1 = new CANSparkMax(Constants.DrivetrainConstants.kDrivetrainMotorR1, MotorType.kBrushless);

    r2 = new CANSparkMax(Constants.DrivetrainConstants.kDrivetrainMotorR2, MotorType.kBrushless);


    leftWheels = new MotorControllerGroup(l1, l2);

    rightWheels = new MotorControllerGroup(r1, r2);

        // sets drivetrain to differential drive

    drivetrain = new DifferentialDrive(leftWheels, rightWheels);
  }

  public void Drive() {

        // sets curvature drive and handles inputs

  drivetrain.curvatureDrive(m_driverController.getLeftX() * 
  Constants.DrivetrainConstants.kDrivetrainForwardConstant, 
  m_driverController.getRightY() * Constants.DrivetrainConstants.kDrivetrainRotateConstant,
   true);
   
  }

  @Override
  public void periodic() {

  }
}
