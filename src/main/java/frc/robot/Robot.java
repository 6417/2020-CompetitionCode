/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Motors motors;
  private ShuffleBoard shuffleBoard;
  private RobotContainer m_robotContainer;

  private double autonomousStartTime = 0;

  public static AHRS ahrs;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
    }

    shuffleBoard = new ShuffleBoard();
    motors = new Motors();
    m_robotContainer = new RobotContainer();

    new Thread (() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(625, 360);
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 625, 360);
      Mat source = new Mat();
      Mat output = new Mat();
      while(!Thread.interrupted()) {
        cvSink.grabFrame(source);
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }

    }).start();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    shuffleBoard.updateShuffleboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
//    CommandScheduler.getInstance().cancelAll();
    motors.disableAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }

    CommandScheduler.getInstance().schedule(Commands.autonomousDrive);

    // Commands.gripperSubsystem.extendProtector(); 
    // Commands.gripperSubsystem.setProtectorExtended(true);

    // autonomousStartTime = System.currentTimeMillis();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {


      // Motors.thrower_motor_upper_shaft_right.set(0.8);

      // if(System.currentTimeMillis() > autonomousStartTime + 7000) {
      //   Motors.tunnel_motor.stopMotor();
      //   Motors.thrower_motor_lower_shaft.stopMotor();
      //   Motors.thrower_motor_upper_shaft_right.stopMotor();
      // } else if(System.currentTimeMillis() > autonomousStartTime + 3000) {
      //   Motors.tunnel_motor.set(Constants.TUNNEL_MOTOR_SPEED_FEEDER);
      //   Motors.thrower_motor_lower_shaft.set(-0.45);
      // }


  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    if(RobotContainer.driveJoystick.getRawButton(1)) {
      Motors.gripper_motor.set(RobotContainer.driveJoystick.getThrottle() * 1);
//      Motors.thrower_motor_lower_shaft.set(-RobotContainer.driveJoystick.getThrottle() * 0.5);
//      Motors.thrower_motor_upper_shaft_left.set(-RobotContainer.driveJoystick.getThrottle() * 0.5);
      Motors.tunnel_motor.set(RobotContainer.driveJoystick.getThrottle() * 0.6);
    } else {
      Motors.gripper_motor.stopMotor();
      Motors.tunnel_motor.stopMotor();
//      Motors.thrower_motor_upper_shaft_right.stopMotor();
//      Motors.thrower_motor_lower_shaft.stopMotor();
    } 

    SmartDashboard.putNumber("GripperSpeed", Motors.gripper_motor.get());
    SmartDashboard.putNumber("Joystick Max Speed", ShuffleBoard.joystickMaxSpeed.getDouble(1));
    SmartDashboard.putBoolean("Pressure Switch", Commands.pneumaticsSubsystem.getPreasureSwitchValue());
  }
}