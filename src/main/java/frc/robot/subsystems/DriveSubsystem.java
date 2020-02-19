/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ShuffleBoard;

public class DriveSubsystem extends SubsystemBase {

  private DifferentialDrive diffdrive;
  private DifferentialDriveOdometry odometry;
  private Pose2d mPose2d;
  private AHRS ahrs;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    diffdrive = new DifferentialDrive(Motors.leftMotors, Motors.rightMotors);
    diffdrive.setRightSideInverted(true);
    super.addChild("Drive Front Right", Motors.drive_motor_front_right);
    super.addChild("Drive Front Left", Motors.drive_motor_front_left);
    super.addChild("Drive Back Right", Motors.drive_motor_back_right);
    super.addChild("Drive Back Left", Motors.drive_motor_back_left);
//    odometry = new DifferentialDriveOdometry(getGyroAngle(), new Pose2d(5.0, 13.5, new Rotation2d()));
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
    Constants.STEERING_WHEEL_USAGE = ShuffleBoard.joystick.getBoolean(false);
  }

  public void arcadeDrive() {
    if(Constants.STEERING_WHEEL_USAGE) {
//turn with wheel      diffdrive.arcadeDrive(-RobotContainer.driveJoystick.getY(), RobotContainer.steerJoystick.getX());
//arcade drive for reverse inverted      


/*
      if(RobotContainer.steerJoystick.getX() < 0) {
        diffdrive.tankDrive((RobotContainer.steerJoystick.getX() + 0.5) * 2 * (-RobotContainer.driveJoystick.getY()), -RobotContainer.driveJoystick.getY());
      } else {
        diffdrive.tankDrive(-RobotContainer.driveJoystick.getY(), (RobotContainer.steerJoystick.getX() - 0.5) * 2 * (RobotContainer.driveJoystick.getY()));
      }
      SmartDashboard.putNumber("Drive Joystick", -RobotContainer.driveJoystick.getY());
      SmartDashboard.putNumber("Steer Joystick", RobotContainer.steerJoystick.getX());
*/


      //        diffdrive.tankDrive(RobotContainer.driveJoystick.getY() - RobotContainer.steerJoystick.getX()  + RobotContainer.steerJoystick.getX(), RobotContainer.driveJoystick.getY() - RobotContainer.steerJoystick.getX() - RobotContainer.steerJoystick.getX());
              //diffdrive.arcadeDrive(-RobotContainer.driveJoystick.getY(), RobotContainer.steerJoystick.getX() * Math.abs(RobotContainer.driveJoystick.getY()) * 2);
//drive with feetpedal      diffdrive.arcadeDrive(RobotContainer.steerJoystick.getZ() - RobotContainer.steerJoystick.getY(), -RobotContainer.steerJoystick.getX() * (RobotContainer.steerJoystick.getZ() - RobotContainer.steerJoystick.getY()));
    } else {
      diffdrive.arcadeDrive(-RobotContainer.driveJoystick.getY() * ShuffleBoard.joystickMaxSpeed.getDouble(1.0), RobotContainer.driveJoystick.getX() * ShuffleBoard.joystickMaxSpeed.getDouble(1.0) * 0.6);
    }
  }
  
  public float getAngle() {
    return ahrs.getYaw();

  }

    
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(-getAngle());
  }

  public void resetAngle() {
    ahrs.reset();
  }




  public void updatePose() {
//    mPose2d = odometry.update(getGyroAngle(), Motors.drive_motor_front_right.getEncoder().getPosition(), Motors.drive_motor_front_right.getEncoder().getPosition());
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
/*    builder.addDoubleProperty("Encoder Front Left", () -> Motors.drive_motor_front_left.getEncoder().getPosition(),
        pos -> Motors.drive_motor_front_left.getEncoder().setPosition((int) pos));
        
    builder.addDoubleProperty("Encoder Back Left", () -> Motors.drive_motor_back_left.getEncoder().getPosition(),
        pos -> Motors.drive_motor_back_left.getEncoder().setPosition((int) pos));

    builder.addDoubleProperty("Encoder Front Right", () -> Motors.drive_motor_front_right.getEncoder().getPosition(),
        pos -> Motors.drive_motor_front_right.getEncoder().setPosition((int) pos));

    builder.addDoubleProperty("Encoder Back Right", () -> Motors.drive_motor_back_right.getEncoder().getPosition(),
        pos -> Motors.drive_motor_back_right.getEncoder().setPosition((int) pos));
  */  }
}
