// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Drivetrain extends SubsystemBase {

  CANSparkMax leftFront;
  CANSparkMax leftBack;
  CANSparkMax rightFront;
  CANSparkMax rightBack;

  DifferentialDrive drivetrain;

  CommandXboxController controller;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    leftFront = new CANSparkMax(0, MotorType.kBrushless);
    leftBack = new CANSparkMax(0, MotorType.kBrushless);
    rightFront = new CANSparkMax(0, MotorType.kBrushless);
    rightBack = new CANSparkMax(0, MotorType.kBrushless);

    rightBack.follow(rightFront);
    leftBack.follow(leftFront);

    rightFront.setInverted(true);

    drivetrain = new DifferentialDrive(leftFront, rightFront);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void drive(CommandXboxController controller) {
    this.controller = controller;

    drivetrain.curvatureDrive
    (controller.getLeftX()*.6, controller.getRightY()*.8, true);

  }

  public void driveSlow(CommandXboxController controller) {

    drivetrain.curvatureDrive
    (controller.getLeftX()*.3, controller.getRightY()*.4, true);

  }
}
