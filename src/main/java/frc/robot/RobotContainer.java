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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
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

      throwerEnableButton.whenPressed(throwerFastCommandGroup);
      //throwerVisionEnableButton = new  JoystickButton(driveJoystick, Constants.SJ_THROWER_VISION_ENABLE_BUTTON_ID);
      //throwerVisionEnableButton.whenPressed(throwerFastCommandGroup);

      // throwerVisionEnableButton.whenPressed(new SetThrowerSpeedCommand(throwerSubsystem, Constants.THROWER_MOTOR_UPPER_SHAFT_LONG_THROW_SPEED).andThen(throwerCommandGroup));

      if(Constants.IS_VISION_SUBSYSTEM_IN_USE) {
         throwerVisionEnableButton = new JoystickButton(driveJoystick, Constants.SJ_THROWER_VISION_ENABLE_BUTTON_ID);
        
         if(Constants.IS_DRIVE_SUBSYSTEM_IN_USE) {
           throwerVisionEnableButton.whenPressed(visionAlignCommandGroup);
         }
       }
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
}
