/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Motors;

public class DriveSubsystem extends SubsystemBase {

  private DifferentialDrive diffdrive;
  private DifferentialDriveOdometry odometry;
  private Pose2d mPose2d;
  private AHRS ahrs;

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
    odometry = new DifferentialDriveOdometry(getGyroAngle(), new Pose2d(5.0, 13.5, new Rotation2d()));
    super.addChild("Drive Front Right", Motors.drive_motor_front_right);
    super.addChild("Drive Front Left", Motors.drive_motor_front_left);
    super.addChild("Drive Back Right", Motors.drive_motor_back_right);
    super.addChild("Drive Back Left", Motors.drive_motor_back_left);

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
    }
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
  public float getAngle() {
    return ahrs.getYaw();
  }

    builder.addDoubleProperty("Encoder Front Right", () -> Motors.drive_motor_front_right.getSelectedSensorPosition(),
        pos -> Motors.drive_motor_front_right.setSelectedSensorPosition((int) pos));
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(-getAngle());
  }

    builder.addDoubleProperty("Encoder Back Left", () -> Motors.drive_motor_back_left.getSelectedSensorPosition(),
        pos -> Motors.drive_motor_back_left.setSelectedSensorPosition((int) pos));
  public void resetAngle() {
    ahrs.reset();
  }

    builder.addDoubleProperty("Encoder Back Right", () -> Motors.drive_motor_back_right.getSelectedSensorPosition(),
        pos -> Motors.drive_motor_back_right.setSelectedSensorPosition((int) pos));
  }



  public void updatePose() {
    mPose2d = odometry.update(getGyroAngle(), Motors.drive_motor_front_right.getSelectedSensorPosition(), Motors.drive_motor_front_right.getSelectedSensorPosition());
  }

   public Pose2d getPose() {
     return mPose2d;
   }

  public void resetPose(Pose2d poseMeters) {
    odometry.resetPosition(poseMeters, getGyroAngle());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Encoder Front Left", () -> Motors.drive_motor_front_left.getSelectedSensorPosition(),
        pos -> Motors.drive_motor_front_left.setSelectedSensorPosition((int) pos));



}
