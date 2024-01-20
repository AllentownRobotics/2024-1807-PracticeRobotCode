// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  Value value;
  DoubleSolenoid collectorExtender;
  TalonFX collectorSpin;
  /** Creates a new Collector. */
  public Collector() {

    collectorExtender = new DoubleSolenoid
    (PneumaticsModuleType.CTREPCM, 0, 0);

    collectorSpin = new TalonFX(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setCollector(Value value) {

    this.value = value;
    collectorExtender.set(value);
    
  }

  public void toggleCollector() {
    if (collectorExtender.get() == Value.kForward) {
      
      collectorExtender.set(Value.kReverse);

    } else {

      collectorExtender.set(Value.kForward);

    }
  }

  public void spinCollectorIn() {
    collectorSpin.set(CollectorConstants.kCollectorSpinSpeed);
  }

  public void spinCollectorOut() {
    collectorSpin.set(-CollectorConstants.kCollectorSpinSpeed);
  }
}
