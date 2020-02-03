/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Motors;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */
  public ClimberSubsystem() {
    super.addChild("Climber Motor Right", Motors.climber_motor_right);
    super.addChild("Climber Motor Left", Motors.climber_motor_left);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb() {
    Motors.climber_motor_right.set(Constants.CLIMBING_MOTOR_SPEED);
  }

  public void setHeight(int encoderTicks) {
    Motors.climber_encoder_right.setPosition(encoderTicks);
  }

  public double getHeight() {
    return Motors.climber_encoder_right.getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Height", () -> getHeight(), ticks -> setHeight((int)ticks));
    builder.addDoubleProperty("Speed", () -> Motors.climber_motor_right.get(), speed -> Motors.climber_motor_right.set(speed));
  }

}
