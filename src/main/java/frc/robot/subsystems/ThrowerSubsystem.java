/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Motors;
import frc.robot.ShuffleBoard;

public class ThrowerSubsystem extends SubsystemBase {

  private boolean visionSupport;

  private final SlewRateLimiter throwerLimiter = new SlewRateLimiter(3);
  private double upperSpeed;

  private double setPoint;

  /**
   * Creates a new ThrowerSubsystem.
   */
  public ThrowerSubsystem() {
    if(Constants.IS_VISION_SUBSYSTEM_IN_USE) {
      visionSupport = true;
    } else {
      visionSupport = false;
    }

    super.addChild("Thrower Upper Left Motor", Motors.thrower_motor_upper_shaft_left);
    super.addChild("Thrower Upper Right Motor", Motors.thrower_motor_upper_shaft_right);
    super.addChild("Thrower Lower Motor", Motors.thrower_motor_lower_shaft);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Ampere 775", Motors.thrower_motor_upper_shaft_right.getOutputCurrent());
    SmartDashboard.putNumber("Volts 775", Motors.thrower_motor_upper_shaft_right.getBusVoltage());
    ShuffleBoard.shooterVelocity.setDouble(Motors.thrower_encoder_right.getVelocity());
    setPoint = ShuffleBoard.throwerUpperMotor.getDouble(0)* 2000;
//    Motors.throwerPIDController.setReference(setPoint, ControlType.kVelocity);
  }

  public void turnReverse() {
    Motors.thrower_motor_lower_shaft.set(Constants.THROWER_MOTOR_REVERSE_SPEED);
    Motors.thrower_motor_upper_shaft_right.set(Constants.THROWER_MOTOR_REVERSE_SPEED);
  }

  public void enableUpperThrower() {
    if(visionSupport == true) {

    } else {
//      upperSpeed = throwerLimiter.calculate(ShuffleBoard.throwerUpperMotor.getDouble(0.0));
      upperSpeed = throwerLimiter.calculate(Constants.THROWER_MOTOR_UPPER_SHAFT_STANDARD_SPEED);
      Motors.thrower_motor_upper_shaft_right.set(upperSpeed);
    }
  }

  public void enableLowerThrower() {
    Motors.thrower_motor_lower_shaft.set(Constants.THROWER_MOTOR_LOWER_SHAFT_STANDARD_SPEED);
//    Motors.thrower_motor_lower_shaft.set(ShuffleBoard.throwerLowerMotor.getDouble(0.0));
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

  public boolean isrunning() {
    if(Motors.thrower_motor_upper_shaft_right.get() > 0 || Motors.thrower_motor_upper_shaft_right.get() < 0) {
      System.out.println("Called True");
      return true;
    } else {
      System.out.println("Called False");
      return false;
    }
  }

}
