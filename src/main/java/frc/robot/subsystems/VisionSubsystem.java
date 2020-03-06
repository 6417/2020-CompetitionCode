/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import ch.team6417.utils.Algorithms;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {

  private DigitalOutput visionLight = new DigitalOutput(1);

  private NetworkTableEntry Distance;
  private NetworkTableEntry Angle;
  private NetworkTableEntry Offset;
  private NetworkTableEntry Target;
  private NetworkTableEntry New;

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
    New = table.getEntry("New");
  }

  @Override
  public void periodic() {
    readData();
    SmartDashboard.putBoolean("Vision Light activated", visionLight.get());
    SmartDashboard.putNumber("AngleToRotate", Math.toDegrees(getCalculatedAngle()));
  }

  public void printValues() {
    SmartDashboard.putBoolean("Target", Target.getBoolean(false));
    SmartDashboard.putNumber("Distance", Distance.getDouble(0));
    SmartDashboard.putNumber("Angle", Angle.getDouble(0));
    SmartDashboard.putNumber("Offset", Offset.getDouble(0));
    SmartDashboard.putBoolean("New", New.getBoolean(false));
    
  }

  public void readData() {
    distance = Distance.getDouble(0);
    angle = Angle.getDouble(0);
    offset = Offset.getDouble(0);
    target = Target.getBoolean(false);
  }

  public double getCalculatedAngle() {
    return Algorithms.scale(RobotContainer.driveJoystick.getThrottle(), -1, 1, -30, 30) ;
    //TODO uncomment the debug angle method
    //    return Math.atan(getOffset() / getDistance());
  }

  public boolean getTarget() {
    return target;
  }

  /**
   * Distance to target plane normal from camera plane
   * @return distance in meters
   */
  public double getDistance() {
    return distance;
  }

  /**
   * Angle between normal of camera plane and target plane
   * @return angle in radians
   */
  public double getAngle() {
    return angle;
  }

  /**
   * Offset from camera center to target
   * @return offset is positive when target is right from camera center
   */
  public double getOffset() {
    return -offset;
  }

  @Deprecated
  public boolean isAligned() {
    //TODO return true when target is ready to shoot at
    return true;
  }

  /**
   * Distance between the center point of the target and
   * where the camera plane normal intersects with target plane
   * @return deviation in Meter
   */
  public double getDeviationToTarget() {
    // TODO do complicated math.
    return 0;
  }

  public boolean newValueReceived() {
    boolean ret = New.getBoolean(false);
    New.setBoolean(false);
    return ret;
  }

  public void toggleVisionLight() {
    visionLight.set(!visionLight.get());
  }

}
