/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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

    SmartDashboard.putNumber("Position Left", Motors.climber_motor_left.getEncoder().getPosition());
    SmartDashboard.putNumber("Position Right", Motors.climber_motor_right.getEncoder().getPosition());
    SmartDashboard.putNumber("Position Difference", positionDifference);
    SmartDashboard.putNumber("Soll speed right", (-RobotContainer.driveJoystick.getY() * 0.5 + positionDifference) * 5700);
    SmartDashboard.putNumber("Soll speed left", (-RobotContainer.driveJoystick.getY() * 0.5 - positionDifference) * 5700);
    }

  public void climb() {
    Motors.climberPIDRight.setReference((-RobotContainer.driveJoystick.getY() * 0.5 + positionDifference) * 5700, ControlType.kVelocity);
    Motors.climberPIDLeft.setReference((-RobotContainer.driveJoystick.getY() * 0.5 - positionDifference) * 5700, ControlType.kVelocity);
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
    positionDifference = Motors.climber_motor_left.getEncoder().getPosition() - Motors.climber_motor_right.getEncoder().getPosition();
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
