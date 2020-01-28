/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Motors;

public class ThrowerSubsystem extends SubsystemBase {

  private boolean visionSupport;

  /**
   * Creates a new ThrowerSubsystem.
   */
  public ThrowerSubsystem() {
    if(Constants.IS_VISION_SUBSYSTEM_IN_USE) {
      visionSupport = true;
    } else {
      visionSupport = false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turnReverse() {
    Motors.thrower_motor_lower_shaft.set(Constants.THROWER_MOTOR_REVERSE_SPEED);
    Motors.thrower_motor_upper_shaft_right.set(Constants.THROWER_MOTOR_REVERSE_SPEED);
  }

  public void enableUpperThrower() {
    if(visionSupport == true) {

    } else {
      Motors.thrower_motor_upper_shaft_right.set(Constants.THROWER_MOTOR_UPPER_SHAFT_STANDARD_SPEED);
    }
  }

  public void enableLowerThrower() {
    Motors.thrower_motor_lower_shaft.set(Constants.THROWER_MOTOR_LOWER_SHAFT_STANDARD_SPEED);
  }

  public void stopThrower() {
    Motors.thrower_motor_lower_shaft.stopMotor();
    Motors.thrower_motor_upper_shaft_right.stopMotor();
  }

  public void setVisionSupport(boolean enable) {
    visionSupport = enable;
  }

  public boolean getVisionSupport() {
    return visionSupport;
  }

}
