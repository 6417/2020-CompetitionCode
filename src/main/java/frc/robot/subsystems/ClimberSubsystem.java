/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;

import ch.team6417.utils.Algorithms;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Motors;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {

  private double positionDifference;

  public enum ClimbType {
    LEFT_UP, LEFT_DOWN, RIGHT_UP, RIGHT_DOWN
  }

  /**
   * Creates a new ClimberSubsystem.
   */
  public ClimberSubsystem() {
    super.addChild("Climber Motor Right", Motors.climber_motor_right);
    super.addChild("Climber Motor Left", Motors.climber_motor_left);
    positionDifference = 0;
  }

  @Override
  public void periodic() {

    checkSafetyStop();
    calculatePositionDifference();
    checkLimits();

    SmartDashboard.putNumber("Position Left", Motors.climber_encoder_left.getPosition());
    SmartDashboard.putNumber("Position Right", Motors.climber_encoder_right.getPosition());
    SmartDashboard.putNumber("Position Difference", positionDifference);
    SmartDashboard.putNumber("Soll speed right", (-RobotContainer.driveJoystick.getY() * 0.5 + positionDifference) * 5700);
    SmartDashboard.putNumber("Soll speed left", (-RobotContainer.driveJoystick.getY() * 0.5 - positionDifference) * 5700);
    }

  public void climb() {
    double setpoint = Algorithms.scale(RobotContainer.driveJoystick.getThrottle(), -1, 1, -130.5, 0.0);
    Motors.climberPIDLeft.setSetpoint(setpoint);
    double difference = Motors.climber_encoder_right.getPosition() - Motors.climber_encoder_left.getPosition();
  //  Motors.climberPIDRight.setReference((-RobotContainer.driveJoystick.getY() * 0.7 + positionDifference) * 5700, ControlType.kVelocity);
  //  Motors.climberPIDLeft.setReference((-RobotContainer.driveJoystick.getY() * 0.7 - positionDifference) * 5700, ControlType.kVelocity);
  //  Motors.climberPIDRight.setReference(setpoint, ControlType.kPosition);
  //  Motors.climberPIDLeft.setReference(setpoint, ControlType.kPosition);
    double boost_right = Motors.climberPIDRight.calculate(difference);
    double boost_left = Motors.climberPIDLeft.calculate(Motors.climber_encoder_left.getPosition());
    double speed_right = boost_left + boost_right;
    double speed_left = boost_left - boost_right;
    double max = Math.max(speed_right, speed_left);
    if(max > 1) {
      speed_left /= max;
      speed_right /= max;
    }
    Motors.climber_motor_right.set(speed_right);
    Motors.climber_motor_left.set(speed_left);
    SmartDashboard.putNumber("boost left", boost_left);
    SmartDashboard.putNumber("boost_right", boost_right);
    SmartDashboard.putNumber("speed left", speed_left);
    SmartDashboard.putNumber("speed right", speed_right);
    SmartDashboard.putNumber("max speed", max);
    SmartDashboard.putNumber("Velocity Error", Motors.climberPIDLeft.getVelocityError());
    System.out.println(setpoint);
  }

  public void climbUpRight() {
    Motors.climber_motor_right.set(-Constants.MANUAL_CLIMB_SPEED);
  }

  public void climbUpLeft() {
    Motors.climber_motor_left.set(-Constants.MANUAL_CLIMB_SPEED);
  }

  public void climbDownRight() {
    Motors.climber_motor_right.set(Constants.MANUAL_CLIMB_SPEED);
  }

  public void climbDownLeft() {
    Motors.climber_motor_left.set(Constants.MANUAL_CLIMB_SPEED);
  }

  public void stopClimber() {
    Motors.climber_motor_left.stopMotor();
    Motors.climber_motor_right.stopMotor();
  }

  public void setHeight(int encoderTicks) {
    Motors.climber_encoder_right.setPosition(encoderTicks);
    Motors.climber_encoder_left.setPosition(encoderTicks);
  }

  public void resetHeight() {
    Motors.climber_encoder_right.setPosition(0);
    Motors.climber_encoder_left.setPosition(0);
  }

  public void resetLeftHeight() {
    Motors.climber_encoder_left.setPosition(0);
  }

  public void resetRightHeight() {
    Motors.climber_encoder_right.setPosition(0);
  }

  public double getHeight() {
    return Motors.climber_encoder_right.getPosition();
  }

  public double calculatePositionDifference() {
    positionDifference = Motors.climber_encoder_left.getPosition() - Motors.climber_encoder_right.getPosition();
    positionDifference = Algorithms.scale(positionDifference, -100, 100, -0.1, 0.1);
    return positionDifference;
  }

  public void checkSafetyStop() {
    if(positionDifference < -Constants.CLIMBER_LEVEL_SAFETY_TICKS || positionDifference > Constants.CLIMBER_LEVEL_SAFETY_TICKS) {
      Motors.climber_motor_left.stopMotor();
      Motors.climber_motor_right.stopMotor();
    }
  }

  public void checkLimits() {
    if(Motors.right_limit.get()) {
      resetRightHeight();
    }

    if(Motors.left_limit.get()) {
      resetLeftHeight();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Height", () -> getHeight(), ticks -> setHeight((int)ticks));
    builder.addDoubleProperty("Speed", () -> Motors.climber_motor_right.get(), speed -> Motors.climber_motor_right.set(speed));
  }

}
