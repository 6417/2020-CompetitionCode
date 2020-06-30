/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.TrajectoryConstants.AutoConstants;
import frc.robot.TrajectoryConstants.DriveConstants;
import frc.robot.commands.vision.SwitchVisionLightCommand;
import frc.robot.commands.vision.VisionAlignCommand;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends Commands {
  // The robot's subsystems and commands are defined here...
  public static Joystick driveJoystick;
  public static Joystick steerJoystick;

  private JoystickButton disableRobotButton;
  private JoystickButton disableAutonomousSystemsButton;
  
  private JoystickButton controlPanelLiftButton;
  private JoystickButton controlPanelTurnButton;

  private JoystickButton flowForwardButton;
  private JoystickButton flowReverseButton;  
  private JoystickButton gripperSoloTurnButton;
  private JoystickButton gripperProtectorButton;
  
  private JoystickButton throwerEnableButton;
  private JoystickButton throwerVisionEnableButton;

  public static JoystickButton manualLeftClimbUpButton;
  public static JoystickButton manualLeftClimbDownButton;
  public static JoystickButton manualRightClimbUpButton;
  public static JoystickButton manualRightClimbDownButton;

  private JoystickButton enableClimbButton;
  

  public RobotContainer() {
    // Configure the button bindings
    configureJoysticks();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureJoysticks() {

    if(Constants.STEERING_WHEEL_USAGE) {
      driveJoystick = new Joystick(Constants.JOYSTICK_DRIVE_ID);
      steerJoystick = new Joystick(Constants.JOYSTICK_STEER_ID);
    } else {
      driveJoystick = new Joystick(Constants.SINGLE_JOYSTICK);
    }
  }

  private void configureButtonBindings() {

    disableRobotButton = new JoystickButton(driveJoystick, Constants.SJ_DISABLE_ROBOT_BUTTON_ID);
    disableAutonomousSystemsButton = new JoystickButton(driveJoystick, Constants.SJ_DISABLE_AUTONOMOUS_SYSTEMS_BUTTON_ID);

    if(disableRobotButton.get()) {
      CommandScheduler.getInstance().cancelAll();
      Motors.disableAll();
    }

    if(disableAutonomousSystemsButton.get()) {
      Constants.AUTONOMOUS_SYSTEMS_ENABLED = !Constants.AUTONOMOUS_SYSTEMS_ENABLED;
    }

    if(Constants.IS_CONTORL_PANEL_SUBSYSTEM_IN_USE) {
      controlPanelLiftButton = new JoystickButton(driveJoystick, Constants.SJ_CONTROL_PANEL_LIFT_BUTTON_ID);
      controlPanelTurnButton = new JoystickButton(driveJoystick, Constants.SJ_CONTROL_PANEL_TURN_BUTTON_ID);

      controlPanelTurnButton.whenPressed(turnControlPanelCommand);
      controlPanelLiftButton.whenPressed(contorlPanelConditionalCommand);
    }

    if(Constants.IS_GRIPPER_SUBSYSTEM_IN_USE) {
      gripperSoloTurnButton = new JoystickButton(driveJoystick, Constants.SJ_GRIPPER_SOLO_TURN_BUTTON_ID);

      gripperSoloTurnButton.whenPressed(gripperSoloTurnConditionalCommand);

      if(Constants.IS_GRIPPER_PROTECTOR_IN_USE) {

        gripperProtectorButton = new JoystickButton(driveJoystick, Constants.SJ_GRIPPER_PROTECTOR_BUTTON_ID);

        gripperProtectorButton.whenPressed(gripperProtectorConditionalCommand);

      }

    }

    if(Constants.IS_THROWER_SUBSYSTEM_IN_USE) {
      throwerEnableButton = new JoystickButton(driveJoystick, Constants.SJ_THROWER_ENABLE_BUTTON_ID);

      throwerEnableButton.whenPressed(throwerCommandGroup);
      throwerVisionEnableButton = new  JoystickButton(driveJoystick, Constants.SJ_THROWER_VISION_ENABLE_BUTTON_ID);
      throwerVisionEnableButton.whenPressed(throwerFastCommandGroup);

      // throwerVisionEnableButton.whenPressed(new SetThrowerSpeedCommand(throwerSubsystem, Constants.THROWER_MOTOR_UPPER_SHAFT_LONG_THROW_SPEED).andThen(throwerCommandGroup));

      // if(Constants.IS_VISION_SUBSYSTEM_IN_USE) {
      //   throwerVisionEnableButton = new JoystickButton(driveJoystick, Constants.SJ_THROWER_VISION_ENABLE_BUTTON_ID);
        
      //   if(Constants.IS_DRIVE_SUBSYSTEM_IN_USE) {
      //     throwerVisionEnableButton.whenPressed(new SequentialCommandGroup(switchVisionLightCommand, visionAlignCommand, new WaitCommand(1), new VisionAlignCommand(visionSubsystem, driveSubsystem), new WaitCommand(1), new VisionAlignCommand(visionSubsystem, driveSubsystem), new SwitchVisionLightCommand(visionSubsystem)));
      //   }
      // }
    }

    //TODO change Statement for flow commands
    if(false == false && Constants.IS_GRIPPER_SUBSYSTEM_IN_USE && Constants.IS_TUNNEL_SUBSYSTEM_IN_USE && Constants.IS_THROWER_SUBSYSTEM_IN_USE) {
      flowForwardButton = new JoystickButton(driveJoystick, Constants.SJ_FLOW_FORWARD_BUTTON_ID);
      flowReverseButton = new JoystickButton(driveJoystick, Constants.SJ_FLOW_REVERSE_BUTTON_ID);
 
      flowForwardButton.whenPressed(flowForwardConditionalCommand);
      flowReverseButton.whenHeld(flowReverseConditionalCommand);
      flowReverseButton.whenReleased(flowStopCommandGroup);
    }

    if(Constants.IS_CLIMBING_SUBSYSTEM_IN_USE) {
      manualLeftClimbDownButton = new JoystickButton(steerJoystick, Constants.SWJ_LEFT_CLIMB_DOWN_BUTTON);
      manualLeftClimbUpButton = new JoystickButton(steerJoystick, Constants.SWJ_LEFT_CLIMB_UP_BUTTON);
      manualRightClimbDownButton = new JoystickButton(steerJoystick, Constants.SWJ_RIGHT_CLIMB_DOWN_BUTTON);
      manualRightClimbUpButton = new JoystickButton(steerJoystick, Constants.SWJ_RIGHT_CLIMB_UP_BUTTON);

      manualLeftClimbDownButton.whenHeld(manualClimbCommand);
      manualLeftClimbUpButton.whenHeld(manualClimbCommand);
      manualRightClimbDownButton.whenHeld(manualClimbCommand);
      manualRightClimbUpButton.whenHeld(manualClimbCommand);

      enableClimbButton = new JoystickButton(driveJoystick, Constants.ENABLE_CLIMB_BUTTON);

      enableClimbButton.whileHeld(climbUPCommand, false);
      enableClimbButton.whenReleased(climbStopCommand);
    }

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand(int command_ID) {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory driveFromLine = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            // new Translation2d(1, 1),
            // new Translation2d(2, 1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
      //  new Pose2d(3, 0, new Rotation2d(0)),
        new Pose2d(1, 0, new Rotation2d(0)),
        // Pass config
        config
    );

      // An example trajectory to follow.  All units in meters.
      Trajectory testReverse = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
          ),
          new Pose2d(1, 0.2, new Rotation2d(0)),
          config
      );


    Trajectory sixBallAutonomousGrab = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0.0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1.6 , 1.5)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5.1, 1.6, new Rotation2d(0.0)),
        // Pass config
        config
    );

    //Reset Pose befor starting this
    Trajectory sixBallAutonomousShoot = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0.0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1.6 , 1.5)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5.2, 1.5, new Rotation2d(0.0)),
        // Pass config
        config
  );

      //Reset Pose befor starting this
      Trajectory fiveBallAutonomousGrab = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0.0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(2.1 , -4.1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3.3, -5.2, new Rotation2d(0.9)),
        // Pass config
        config
  );

  RamseteCommand ramseteCommand_default = new RamseteCommand(
    sixBallAutonomousGrab,
    Commands.driveSubsystem::getReversePose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts,
                               DriveConstants.kvVoltSecondsPerMeter,
                               DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    Commands.driveSubsystem::getReverseWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    Commands.driveSubsystem::reverseTankDriveVolts
    );

    RamseteCommand ramseteCommand_1 = new RamseteCommand(
        sixBallAutonomousGrab,
        Commands.driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        Commands.driveSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        Commands.driveSubsystem::tankDriveVolts
        // Commands.driveSubsystem
    );

    RamseteCommand ramseteCommand_2 = new RamseteCommand(
      sixBallAutonomousShoot,
      Commands.driveSubsystem::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      Commands.driveSubsystem::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      Commands.driveSubsystem::tankDriveVolts
      // Commands.driveSubsystem
  );

  RamseteCommand ramseteCommand_3 = new RamseteCommand(
    fiveBallAutonomousGrab,
    Commands.driveSubsystem::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts,
                               DriveConstants.kvVoltSecondsPerMeter,
                               DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    Commands.driveSubsystem::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    Commands.driveSubsystem::tankDriveVolts
    // Commands.driveSubsystem
);

  if(command_ID == 1) {
    // Run path following command, then stop at the end.
    return ramseteCommand_1.andThen(() -> Commands.driveSubsystem.tankDriveVolts(0, 0));
  } else if(command_ID == 2) {
    // Run path following command, then stop at the end.
    return ramseteCommand_2.andThen(() -> Commands.driveSubsystem.tankDriveVolts(0, 0));
  } else if(command_ID == 3) {
    return ramseteCommand_3.andThen(() -> Commands.driveSubsystem.tankDriveVolts(0, 0));
  } else {
    // Run path following command, then stop at the end.
    return ramseteCommand_default.andThen(() -> Commands.driveSubsystem.tankDriveVolts(0, 0));
  }
  }
}
