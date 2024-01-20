// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IndexerCommands.IndexForward;

public class Shooter extends SubsystemBase {

  public CANSparkMax flywheel;
  DoubleSolenoid hood;

  Value value;

  SparkPIDController pidController;

  SparkRelativeEncoder encoder;

  Indexer indexer;

  /** Creates a new Shooter. */
  public Shooter() {

    flywheel = new CANSparkMax(0, MotorType.kBrushless);
    hood = new DoubleSolenoid
    (PneumaticsModuleType.CTREPCM, 0, 0);
    
    pidController = flywheel.getPIDController();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFlywheel() {

    pidController.setReference(1000, ControlType.kVelocity);

  }

  public void fireGamePiece() {
    new IndexForward(indexer);
  }


  public void hoodSet(Value value) {

    this.value = value;
    hood.set(value);

  }

  public void hoodToggle() {
    if (hood.get() == Value.kForward) {

      hood.set(Value.kReverse);

    } else {
      
      hood.set(Value.kForward);

    }
  }
}
