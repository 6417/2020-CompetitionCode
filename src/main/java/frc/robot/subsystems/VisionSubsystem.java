/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private NetworkTableEntry Distance;
  private NetworkTableEntry Angle;
  private NetworkTableEntry Offset;
  private NetworkTableEntry Target;

  private double distance;
  private double angle;
  private double offset;
  private boolean target;
    
  /**
   * Creates a new VisionSubsystem.
   */
  public VisionSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable table = inst.getTable("vision");

    Distance = table.getEntry("Distance");
    Angle = table.getEntry("Angle");
    Offset = table.getEntry("XOffset");
    Target = table.getEntry("Target");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void printValues() {
    SmartDashboard.putBoolean("Target", Target.getBoolean(false));
    SmartDashboard.putNumber("Distance", Distance.getDouble(0));
    SmartDashboard.putNumber("Angle", Angle.getDouble(0));
    SmartDashboard.putNumber("Offset", Offset.getDouble(0));
  }

  public void readData() {
    distance = Distance.getDouble(0);
    angle = Angle.getDouble(0);
    offset = Offset.getDouble(0);
    target = Target.getBoolean(false);
  }

  public boolean getTarget() {
    return target;
  }

  public double getDistance() {
    return distance;
  }

  public double getAngle() {
    return angle;
  }

  public double getOffset() {
    return offset;
  }

  public boolean isAligned() {
    //TODO return true when target is ready to shoot at
    return false;
  }

}
