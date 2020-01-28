/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends Commands {
  // The robot's subsystems and commands are defined here...
  private final Joystick driveJoystick = new Joystick(Constants.SINGLE_JOYSTICK);

  
  private JoystickButton controlPanelLiftButton;
  private JoystickButton controlPanelTurnButton;

  private JoystickButton flowForwardButton;
  private JoystickButton flowReverseButton;  
  private JoystickButton gripperSoloTurnButton;
  
  private JoystickButton throwerEnableButton;
  

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    if(Constants.IS_CONTORL_PANEL_SUBSYSTEM_IN_USE) {
      controlPanelLiftButton = new JoystickButton(driveJoystick, Constants.SJ_CONTROL_PANEL_LIFT_BUTTON_ID);
      controlPanelTurnButton = new JoystickButton(driveJoystick, Constants.SJ_CONTROL_PANEL_TURN_BUTTON_ID);

      controlPanelTurnButton.whenPressed(turnControlPanelCommand);
      controlPanelLiftButton.whenPressed(contorlPanelConditionalCommand);
    }

    if(Constants.IS_GRIPPER_SUBSYSTEM_IN_USE) {
      gripperSoloTurnButton = new JoystickButton(driveJoystick, Constants.SJ_GRIPPER_SOLO_TURN_BUTTON_ID);

      gripperSoloTurnButton.whenHeld(gripperSoloTurnConditionalCommand);
    }

    if(Constants.IS_THROWER_SUBSYSTEM_IN_USE) {
      throwerEnableButton = new JoystickButton(driveJoystick, Constants.SJ_THROWER_ENABLE_BUTTON_ID);

      throwerEnableButton.whenPressed(throwerExtrude);
    }

    if(Constants.IS_GRIPPER_SUBSYSTEM_IN_USE && Constants.IS_TUNNEL_SUBSYSTEM_IN_USE && Constants.IS_THROWER_SUBSYSTEM_IN_USE) {
      flowForwardButton = new JoystickButton(driveJoystick, Constants.SJ_FLOW_FORWARD_BUTTON_ID);
      flowReverseButton = new JoystickButton(driveJoystick, Constants.SJ_FLOW_REVERSE_BUTTON_ID);
 
      flowForwardButton.whenPressed(flowForwardConditionalCommand);
      flowReverseButton.whenHeld(flowReverseConditionalCommand);
    }

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
/*  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }*/
}
