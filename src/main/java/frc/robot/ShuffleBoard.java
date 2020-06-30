/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Add your docs here.
 */
public class ShuffleBoard {

    public ShuffleBoard() {}
    
    private static ShuffleboardTab speeds = Shuffleboard.getTab("2020Speeds");
    private static ShuffleboardTab competition = Shuffleboard.getTab("Competition");
    private static ShuffleboardTab tab = Shuffleboard.getTab("extra");

    //Setters
    public static NetworkTableEntry throwerUpperMotor =
        speeds.add("Thrower Upper Motor speed", Constants.THROWER_MOTOR_UPPER_SHAFT_STANDARD_SPEED)
        .getEntry();

    public static NetworkTableEntry throwerLowerMotor =
        speeds.add("Thrower Lower Motor speed", Constants.THROWER_MOTOR_LOWER_SHAFT_STANDARD_SPEED)
        .getEntry();

    public static NetworkTableEntry throwerReversMotorSpeed =
        speeds.add("Thrower Reverse Motor speed", Constants.THROWER_MOTOR_REVERSE_SPEED)
        .getEntry();



    public static NetworkTableEntry tunnelMotorForward =
        speeds.add("Tunnel Motor Forward speed", Constants.TUNNEL_MOTOR_SPEED_FORWARD)
        .getEntry();

    public static NetworkTableEntry tunnelMotorReverse = 
        speeds.add("Tunnel Motor Reverse speed", Constants.TUNNEL_MOTOR_SPEED_REVERSE)
        .getEntry();

    public static NetworkTableEntry tunnelMotorFeederSpeed = 
        speeds.add("Tunnel Motor feeder speed", Constants.TUNNEL_MOTOR_SPEED_FEEDER)
        .getEntry();



    public static NetworkTableEntry gripperMotorSpeed =
        speeds.add("Gripper Forward Motor speed", Constants.GRIPPER_MOTOR_SPEED_FORWARD)
        .getEntry();

    public static NetworkTableEntry gripperMotorReverseSpeed = 
        speeds.add("Gripper Reverse speed", Constants.GRIPPER_MOTOR_SPEED_REVERSE)
        .getEntry();



    public static NetworkTableEntry controlPanelTurnSpeed =
        speeds.add("Control Panel Motor Turn Speed", Constants.CONTROL_PANEL_TURN_SPEED)
        .getEntry();


    //getters
    public static NetworkTableEntry shooterVelocity =
        competition.add("Shooter Velocity", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    public static NetworkTableEntry gripperVelocity =
        competition.add("Gripper Velocity", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    public static NetworkTableEntry visionLight =
        competition.add("Vision Light", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    public static NetworkTableEntry joystick =
        tab.add("Steering Wheel Joystick", true)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();

        public static NetworkTableEntry joystickMaxSpeed =
        tab.add("Joystick Max Speed", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();

    public void updateShuffleboard() {
        Constants.THROWER_MOTOR_UPPER_SHAFT_STANDARD_SPEED = throwerUpperMotor.getDouble(Constants.THROWER_MOTOR_UPPER_SHAFT_STANDARD_SPEED);
        Constants.THROWER_MOTOR_LOWER_SHAFT_STANDARD_SPEED = throwerLowerMotor.getDouble(Constants.THROWER_MOTOR_LOWER_SHAFT_STANDARD_SPEED);
        Constants.THROWER_MOTOR_REVERSE_SPEED = throwerReversMotorSpeed.getDouble(Constants.THROWER_MOTOR_REVERSE_SPEED);
       
//        Constants.GRIPPER_MOTOR_SPEED_FORWARD = gripperMotorSpeed.getDouble(Constants.GRIPPER_MOTOR_SPEED_FORWARD);
//        Constants.GRIPPER_MOTOR_SPEED_REVERSE = gripperMotorReverseSpeed.getDouble(Constants.GRIPPER_MOTOR_SPEED_REVERSE);

//        Constants.TUNNEL_MOTOR_SPEED_FEEDER = tunnelMotorFeederSpeed.getDouble(Constants.TUNNEL_MOTOR_SPEED_FEEDER);
//        Constants.TUNNEL_MOTOR_SPEED_FORWARD = tunnelMotorForward.getDouble(Constants.GRIPPER_MOTOR_SPEED_FORWARD);
//        Constants.TUNNEL_MOTOR_SPEED_REVERSE = tunnelMotorReverse.getDouble(Constants.GRIPPER_MOTOR_SPEED_REVERSE);

//        Constants.CONTROL_PANEL_TURN_SPEED = controlPanelTurnSpeed.getDouble(Constants.CONTROL_PANEL_TURN_SPEED);
    }

}
