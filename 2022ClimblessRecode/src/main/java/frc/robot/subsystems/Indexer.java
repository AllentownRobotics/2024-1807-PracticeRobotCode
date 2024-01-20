// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  TalonFX indexLower;
  TalonFX indexHigher;
  
  /** Creates a new Indexer. */
  public Indexer() {

    indexLower = new TalonFX(0);
    indexHigher = new TalonFX(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void indexUp() {

    indexLower.set(-.3);
    indexHigher.set(.3);

  }

  public void indexDown() {

    indexLower.set(.3);
    indexHigher.set(-.3);
    
  }
}
