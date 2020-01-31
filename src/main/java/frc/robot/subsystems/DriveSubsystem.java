/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Motors;

public class DriveSubsystem extends SubsystemBase {

  private DifferentialDrive diffdrive;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    diffdrive = new DifferentialDrive(Motors.drive_motor_front_left, Motors.drive_motor_front_right);
    diffdrive.setRightSideInverted(true);
    super.addChild("Drive Front Right", Motors.drive_motor_front_right);
    super.addChild("Drive Front Left", Motors.drive_motor_front_left);
    super.addChild("Drive Back Right", Motors.drive_motor_back_right);
    super.addChild("Drive Back Left", Motors.drive_motor_back_left);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Encoder Front Left", () -> Motors.drive_motor_front_left.getSelectedSensorPosition(),
        pos -> Motors.drive_motor_front_left.setSelectedSensorPosition((int) pos));

    builder.addDoubleProperty("Encoder Front Right", () -> Motors.drive_motor_front_right.getSelectedSensorPosition(),
        pos -> Motors.drive_motor_front_right.setSelectedSensorPosition((int) pos));

    builder.addDoubleProperty("Encoder Back Left", () -> Motors.drive_motor_back_left.getSelectedSensorPosition(),
        pos -> Motors.drive_motor_back_left.setSelectedSensorPosition((int) pos));

    builder.addDoubleProperty("Encoder Back Right", () -> Motors.drive_motor_back_right.getSelectedSensorPosition(),
        pos -> Motors.drive_motor_back_right.setSelectedSensorPosition((int) pos));
  }



}
