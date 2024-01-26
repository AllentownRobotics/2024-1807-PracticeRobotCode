// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry t3d;

  double x;
  double y;
  double area;

  double x3d;
  double y3d;
  double z3d;
  double[] values3d;

  /** Creates a new Limelight. */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);




    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    t3d = table.getEntry("camerapose_targetspace");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    values3d = t3d.getDoubleArray(new double[6]);

    x3d = values3d[0];
    y3d = values3d[1];
    z3d = values3d[2];

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Limelight3dX", x3d);
    SmartDashboard.putNumber("Limelight3dY", y3d);
    SmartDashboard.putNumber("Limelight3dZ", z3d);
  }

  public double getX()
  {
    return x;
  }
}
