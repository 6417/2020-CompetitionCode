/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands;
import frc.robot.Constants;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ShuffleBoard;

public class ThrowerSubsystem extends SubsystemBase {

  private DigitalInput stackSensor;

  private boolean visionSupport, stackClear, stackPushing, timeStarted;

  private long startTime;

  private final SlewRateLimiter throwerLimiter = new SlewRateLimiter(3);
  private double upperSpeed;

  private double setPoint;

  /**
   * Creates a new ThrowerSubsystem.
   */
  public ThrowerSubsystem() {
    stackSensor = new DigitalInput(0);

    stackClear = false;
    stackPushing = false;
    timeStarted = false;

    if(Constants.IS_VISION_SUBSYSTEM_IN_USE) {
      visionSupport = true; //TODO enable vision Support
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
    if(Motors.thrower_motor_upper_shaft_right.getEncoder().getVelocity() != 0) {
      ShuffleBoard.shooterVelocity.setBoolean(true);
    } else {
      ShuffleBoard.shooterVelocity.setBoolean(false);
    }    
    setPoint = ShuffleBoard.throwerUpperMotor.getDouble(0)* 2000;
//    Motors.throwerPIDController.setReference(setPoint, ControlType.kVelocity);
    feedStack();
  }

  public void turnReverse() {
    Motors.thrower_motor_lower_shaft.set(Constants.THROWER_MOTOR_REVERSE_SPEED);
    Motors.thrower_motor_upper_shaft_right.set(Constants.THROWER_MOTOR_REVERSE_SPEED);
  }

  public void enableUpperThrower() {
    if(visionSupport == true && Commands.visionSubsystem.isAligned() == true) {
      double speed = 0.0003 * Commands.visionSubsystem.getDistance() + 0.711;
      SmartDashboard.putNumber("Shooter speed", speed);
      upperSpeed = throwerLimiter.calculate(speed);
      Motors.thrower_motor_upper_shaft_right.set(upperSpeed);
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
    stackClear = true;
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

  public void feedStack() {
    if(stackClear == true && Motors.thrower_motor_upper_shaft_right.get() == 0 && Motors.tunnel_motor.getMotorOutputPercent() != 0) {
      System.out.println("time to take it up");
      if(stackSensor.get() == false || stackPushing == true) {
        System.out.println("Sensor got or stack is pushing");
        stackPushing = true;
        if(timeStarted == false) {
          startTime = System.currentTimeMillis();
          Motors.thrower_motor_lower_shaft.getEncoder().setPosition(0);
        }
        timeStarted = true;
        if(System.currentTimeMillis() >= startTime + 700) {
          System.out.println("waited for time");
          Motors.thrower_motor_lower_shaft.set(-0.1);
          if(Motors.thrower_motor_lower_shaft.getEncoder().getPosition() < -2) {
            System.out.println("stopped");
            Motors.thrower_motor_lower_shaft.stopMotor();
            stackClear = false;
            stackPushing = false;
            timeStarted = false;
          }
        }
      }
    }
  }

}
