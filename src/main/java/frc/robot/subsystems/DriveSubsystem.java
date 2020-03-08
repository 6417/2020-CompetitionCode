/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import ch.team6417.utils.Algorithms;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands;
import frc.robot.Constants;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ShuffleBoard;
import frc.robot.TrajectoryConstants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private DifferentialDrive diffdrive;
  private DifferentialDriveOdometry odometry;
  private Pose2d mPose2d;
  private AHRS ahrs;
  public PIDController turnController;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {


    Robot.ahrs.reset();
    resetEncoders();

    diffdrive = new DifferentialDrive(Motors.leftMotors, Motors.rightMotors);
    // diffdrive = new DifferentialDrive(Motors.drive_motor_front_right, Motors.drive_motor_front_left);
    diffdrive.setRightSideInverted(true);

    // Sets the distance per pulse for the encoders
    Motors.drive_encoder_front_left.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    Motors.drive_encoder_front_left.setVelocityConversionFactor(DriveConstants.kWheelRPMinMeterPerSeconds);
    Motors.drive_encoder_front_right.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    Motors.drive_encoder_front_right.setVelocityConversionFactor(DriveConstants.kWheelRPMinMeterPerSeconds);

    SmartDashboard.putNumber("Left Velocity Conversion Factor", Motors.drive_encoder_front_left.getVelocityConversionFactor());
    SmartDashboard.putNumber("Left Position Conversion Factor", Motors.drive_encoder_front_left.getPositionConversionFactor());
    SmartDashboard.putNumber("Right Velocity Conversion Factor", Motors.drive_encoder_front_right.getVelocityConversionFactor());
    SmartDashboard.putNumber("Right Position Conversion Factor", Motors.drive_encoder_front_right.getPositionConversionFactor());

    
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
    
    SmartDashboard.putNumber("Velocity Left Encoder", Motors.drive_encoder_front_left.getVelocity());
    SmartDashboard.putNumber("Position Left Encoder", Motors.drive_encoder_front_left.getPosition());
    SmartDashboard.putNumber("Velocity Right Encoder", -Motors.drive_encoder_front_right.getVelocity());
    SmartDashboard.putNumber("Position Right Encoder", -Motors.drive_encoder_front_right.getPosition());
    SmartDashboard.putString("Pose", odometry.getPoseMeters().toString());

  }

  public void arcadeDrive() {
    double drive;
    double steer;
    if(Constants.IS_CONTORL_PANEL_SUBSYSTEM_IN_USE) {
      double maxSpeed = Commands.controlPanelSubsystem.influenceDrive();
      drive = Math.min(RobotContainer.driveJoystick.getY(), maxSpeed);
      // steer = -Algorithms.scale(RobotContainer.driveJoystick.getX(), -1, 1, -maxSpeed, maxSpeed);
      steer = -RobotContainer.driveJoystick.getX();
    } else {
      drive = RobotContainer.driveJoystick.getY();
      steer = -RobotContainer.driveJoystick.getX();
    }

    if(Constants.STEERING_WHEEL_USAGE) {
//turn with wheel      diffdrive.arcadeDrive(-RobotContainer.driveJoystick.getY(), RobotContainer.steerJoystick.getX());
//arcade drive for reverse inverted      



      if(-RobotContainer.steerJoystick.getX() < 0) {
        diffdrive.tankDrive((-RobotContainer.steerJoystick.getX() + 0.5) * 2 * (drive), drive);
      } else {
        diffdrive.tankDrive(drive, (-RobotContainer.steerJoystick.getX() - 0.5) * 2 * (-drive));
      }
      SmartDashboard.putNumber("Drive Joystick", drive);
      SmartDashboard.putNumber("Steer Joystick", -RobotContainer.steerJoystick.getX());



      //        diffdrive.tankDrive(RobotContainer.driveJoystick.getY() - RobotContainer.steerJoystick.getX()  + RobotContainer.steerJoystick.getX(), RobotContainer.driveJoystick.getY() - RobotContainer.steerJoystick.getX() - RobotContainer.steerJoystick.getX());
              //diffdrive.arcadeDrive(-RobotContainer.driveJoystick.getY(), RobotContainer.steerJoystick.getX() * Math.abs(RobotContainer.driveJoystick.getY()) * 2);
//drive with feetpedal      diffdrive.arcadeDrive(RobotContainer.steerJoystick.getZ() - RobotContainer.steerJoystick.getY(), -RobotContainer.steerJoystick.getX() * (RobotContainer.steerJoystick.getZ() - RobotContainer.steerJoystick.getY()));
    } else {
      diffdrive.arcadeDrive(drive * ShuffleBoard.joystickMaxSpeed.getDouble(1.0), steer * ShuffleBoard.joystickMaxSpeed.getDouble(1.0) * 0.6);
    }
  }

  public void arcadeDrive(double xSpeed, double zRotate) {
    // zRotate *= -0.001;
    // zRotate += (2 * Commands.visionSubsystem.getCalculatedAngle());
    // zRotate /= 3;
    zRotate *= -1.5;
    SmartDashboard.putNumber("zRotate: ", zRotate);

    zRotate = Math.min(Math.max(zRotate, -25), 25);


    if(zRotate > 0) {
      zRotate = Algorithms.scale(zRotate, 0, 25, 0.02, 0.15);
      // if(zRotate < 0.2) {
      //   zRotate = 0.2;
      // }
    } else {
      zRotate = Algorithms.scale(zRotate, -25, 0, -0.15, -0.02);
      // if(zRotate > -0.2) {
      //   zRotate=-0.2;
      // }
    }

    SmartDashboard.putNumber("dirve zRotate: ", zRotate);

    // Motors.leftMotors.set(zRotate);
    // Motors.rightMotors.set(zRotate);
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(Motors.drive_encoder_front_left.getVelocity(), -Motors.drive_encoder_front_right.getVelocity());
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

      Motors.drive_encoder_front_left.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
      Motors.drive_encoder_front_left.setVelocityConversionFactor(DriveConstants.kWheelRPMinMeterPerSeconds);
      Motors.drive_encoder_front_right.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
      Motors.drive_encoder_front_right.setVelocityConversionFactor(DriveConstants.kWheelRPMinMeterPerSeconds);
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

  public void resetPose() {
    resetEncoders();
    odometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0)), Rotation2d.fromDegrees(Robot.ahrs.getYaw()));
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    Motors.leftMotors.setVoltage(leftVolts);
    Motors.rightMotors.setVoltage(-rightVolts);
    diffdrive.feed();
  }

    /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (Motors.drive_encoder_front_left.getPosition() + (-Motors.drive_encoder_front_right.getPosition())) / 2.0;
  }

    /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    diffdrive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(ahrs.getYaw(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return ahrs.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
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
