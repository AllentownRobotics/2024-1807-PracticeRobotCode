// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  DoubleSolenoid claw;
  public Rev2mDistanceSensor distance;
  
  /** Creates a new Claw. */
  public Claw() {

    claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    distance = new Rev2mDistanceSensor(Port.kMXP);
    distance.setDistanceUnits(Unit.kInches);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleClaw() {

    if (claw.get().equals(Value.kForward)) {

      claw.set(Value.kReverse);

    } else {
      
      claw.set(Value.kForward);
    }
  }

  public void setClaw(Value value) {
    claw.set(value);
  }
}
