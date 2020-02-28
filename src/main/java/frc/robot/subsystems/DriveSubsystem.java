/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import ch.team6417.utils.Algorithms;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands;
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


    Robot.ahrs.reset();
    resetEncoders();

    diffdrive = new DifferentialDrive(Motors.leftMotors, Motors.rightMotors);
    diffdrive.setRightSideInverted(true);
    super.addChild("Drive Front Right", Motors.drive_motor_front_right);
    super.addChild("Drive Front Left", Motors.drive_motor_front_left);
    super.addChild("Drive Back Right", Motors.drive_motor_back_right);
    super.addChild("Drive Back Left", Motors.drive_motor_back_left);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()), new Pose2d(0, 0, new Rotation2d(0)));
    super.addChild("Drive Front Right", Motors.drive_motor_front_right);
    super.addChild("Drive Front Left", Motors.drive_motor_front_left);
    super.addChild("Drive Back Right", Motors.drive_motor_back_right);
    super.addChild("Drive Back Left", Motors.drive_motor_back_left);

  }

  @Override
  public void periodic() {
    Constants.STEERING_WHEEL_USAGE = ShuffleBoard.joystick.getBoolean(true);
    odometry.update(Rotation2d.fromDegrees(- getAngle()), getEncoderLeftMetric(), getEncoderRightMetric());
  }

  public void arcadeDrive() {
    double drive;
    double steer;
    if(Constants.IS_CONTORL_PANEL_SUBSYSTEM_IN_USE) {
      double maxSpeed = Commands.controlPanelSubsystem.influenceDrive();
      drive = Math.min(-RobotContainer.driveJoystick.getY(), maxSpeed);
      steer = Algorithms.scale(RobotContainer.driveJoystick.getX(), -1, 1, -maxSpeed, maxSpeed);
    } else {
      drive = -RobotContainer.driveJoystick.getY();
      steer = RobotContainer.driveJoystick.getX();
    }

    if(Constants.STEERING_WHEEL_USAGE) {
//turn with wheel      diffdrive.arcadeDrive(-RobotContainer.driveJoystick.getY(), RobotContainer.steerJoystick.getX());
//arcade drive for reverse inverted      



      if(RobotContainer.steerJoystick.getX() < 0) {
        diffdrive.tankDrive((RobotContainer.steerJoystick.getX() + 0.5) * 2 * (drive), drive);
      } else {
        diffdrive.tankDrive(drive, (RobotContainer.steerJoystick.getX() - 0.5) * 2 * (-drive));
      }
      SmartDashboard.putNumber("Drive Joystick", drive);
      SmartDashboard.putNumber("Steer Joystick", RobotContainer.steerJoystick.getX());



      //        diffdrive.tankDrive(RobotContainer.driveJoystick.getY() - RobotContainer.steerJoystick.getX()  + RobotContainer.steerJoystick.getX(), RobotContainer.driveJoystick.getY() - RobotContainer.steerJoystick.getX() - RobotContainer.steerJoystick.getX());
              //diffdrive.arcadeDrive(-RobotContainer.driveJoystick.getY(), RobotContainer.steerJoystick.getX() * Math.abs(RobotContainer.driveJoystick.getY()) * 2);
//drive with feetpedal      diffdrive.arcadeDrive(RobotContainer.steerJoystick.getZ() - RobotContainer.steerJoystick.getY(), -RobotContainer.steerJoystick.getX() * (RobotContainer.steerJoystick.getZ() - RobotContainer.steerJoystick.getY()));
    } else {
      diffdrive.arcadeDrive(drive * ShuffleBoard.joystickMaxSpeed.getDouble(1.0), steer * ShuffleBoard.joystickMaxSpeed.getDouble(1.0) * 0.6);
    }
  }
  
  public float getAngle() {
    return Robot.ahrs.getYaw();
  }

    
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(-getAngle());
  }

  public void resetAngle() {
    ahrs.reset();
  }

  private void resetEncoders() {
    Motors.drive_encoder_back_right.setPosition(0);
    Motors.drive_encoder_back_left.setPosition(0);
    Motors.drive_encoder_front_right.setPosition(0);
    Motors.drive_encoder_front_left.setPosition(0);
  }

  public double getEncoderLeftMetric() {
    return (Motors.drive_encoder_back_left.getPosition()/Constants.GEARBOX_TRANSLATION) * Constants.WHEEL_CIRCUMFERENCE;
  }

  public double getEncoderRightMetric() {
    return -(Motors.drive_encoder_back_right.getPosition()/Constants.GEARBOX_TRANSLATION) * Constants.WHEEL_CIRCUMFERENCE;
  }

   public Pose2d getPose() {
     return odometry.getPoseMeters();
   }

  public void resetPose(Pose2d poseMeters) {
    odometry.resetPosition(poseMeters, getGyroAngle());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    if(!Constants.TEST_ROBOT) {
    builder.addDoubleProperty("Encoder Front Left", () -> Motors.drive_encoder_front_left.getPosition(),
        pos -> Motors.drive_encoder_front_left.setPosition((int) pos));
        
    builder.addDoubleProperty("Encoder Back Left", () -> Motors.drive_encoder_back_left.getPosition(),
        pos -> Motors.drive_encoder_back_left.setPosition((int) pos));

    builder.addDoubleProperty("Encoder Front Right", () -> Motors.drive_encoder_front_right.getPosition(),
        pos -> Motors.drive_encoder_front_right.setPosition((int) pos));

    builder.addDoubleProperty("Encoder Back Right", () -> Motors.drive_encoder_back_right.getPosition(),
        pos -> Motors.drive_encoder_back_right.setPosition((int) pos));

    builder.addStringProperty("Pose", () -> (getPose().toString()), null);

    builder.addStringProperty("Rotation", () -> getPose().getRotation().toString(), null);

    builder.addDoubleProperty("Navx", () -> getAngle(), null);
    }

  }
}
