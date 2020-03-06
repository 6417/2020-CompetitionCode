/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    /* Joystick usage */
    public static boolean STEERING_WHEEL_USAGE = true;
    public static final boolean TEST_ROBOT = false; //changes the initialisation of the Motor Contorllers and add the type to the controller groups
    public static boolean AUTONOMOUS_SYSTEMS_ENABLED = true;

    public static final boolean IS_GRIPPER_PROTECTOR_IN_USE = true;

    /* Subsystems in use */
    public static final boolean IS_DRIVE_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_CONTORL_PANEL_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_PNEUMATICS_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_GRIPPER_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_THROWER_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_TUNNEL_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_CLIMBING_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_VISION_SUBSYSTEM_IN_USE = false;

    /* PCM CAN ID */
    public static final int PCM_CAN_ID = 30;

    /* Motor ID */
    public static final int MOTOR_DRIVE_FRONT_RIGHT_ID = 21;
    public static final int MOTOR_DRIVE_FRONT_LEFT_ID = 10;
    public static final int MOTOR_DRIVE_BACK_RIGHT_ID = 13;
    public static final int MOTOR_DRIVE_BACK_LEFT_ID = 11;

    public static final int MOTOR_CONTROL_PANEL_ID = 16;

    public static final int MOTOR_GRIPPER_ID = 14;

    public static final int MOTOR_LOWER_THROWER_SHAFT_ID = 12;
    public static final int MOTOR_UPPER_THROWER_SHAFT_LEFT_ID = 23; 
    public static final int MOTOR_UPPER_THROWER_SHAFT_RIGHT_ID = 22;

    public static final int MOTOR_TUNNEL_ID = 15;

    public static final int MOTOR_CLIMBER_LEFT_ID = 24;
    public static final int MOTOR_CLIMBER_RIGHT_ID = 25;

    /* Solenoid Ports */
    public static final int CONTROL_PANEL_LIFT_SOLENOID_EXTEND_ID = 2;
    public static final int CONTROL_PANEL_LIFT_SOLENOID_RETRACT_ID = 3;
    public static final int CONTROL_PANEL_DAMPING_SOLENOID_EXTEND_ID = 0;
    public static final int CONTROL_PANEL_DAMPING_SOLENOID_RETRACT_ID = 1;

    public static final int GRIPPER_PROTECTOR_SOLENOID_RETRACT_ID = 4;
    public static final int GRIPPER_PROTECTOR_SOLENOID_EXTEND_ID = 5;

    public static final int GRIPPER_SOLENOID_EXTEND_ID = 6;
    public static final int GRIPPER_SOLENOID_RETRACT_ID = 7;

    /* Sensors */
    public static final I2C.Port CONTROL_PANEL_COLOR_SENSOR_I2C_PORT = I2C.Port.kOnboard;

    public static final int CONTROL_PANEL_LIFT_BOTTOM_REED_DI_ID = 0;
    public static final int CONTROL_PANEL_LIFT_TOP_REED_DI_ID = 1;
    public static final int CONTROL_PANEL_DAMPING_BACK_REED_DI_ID = 2;
    public static final int CONTROL_PANEL_DAMPING_FRONT_REED_DI_ID = 3;

    /* Joystick Port */
    public static final int JOYSTICK_STEER_ID = 1;
    public static final int JOYSTICK_DRIVE_ID = 0;

    public static final int SINGLE_JOYSTICK = 0;

    /* Joystick Button Ports for single controller */
    public static final int SJ_CONTROL_PANEL_LIFT_BUTTON_ID = 4;
    public static final int SJ_CONTROL_PANEL_TURN_BUTTON_ID = 6;

    public static final int SJ_FLOW_FORWARD_BUTTON_ID = 2;
    public static final int SJ_FLOW_REVERSE_BUTTON_ID = 11;
    public static final int SJ_GRIPPER_SOLO_TURN_BUTTON_ID = 1;
    public static final int SJ_GRIPPER_PROTECTOR_BUTTON_ID = 9;
    public static final int SJ_THROWER_ENABLE_BUTTON_ID = 5;
    public static final int SJ_THROWER_VISION_ENABLE_BUTTON_ID = 3;
    public static final int SJ_DISABLE_ROBOT_BUTTON_ID = 7;
    public static final int SJ_DISABLE_AUTONOMOUS_SYSTEMS_BUTTON_ID = 8;

    public static final int ENABLE_CLIMB_BUTTON = 10;

//    public static final int ENABLE_CLIMB_BUTTON = 12;

    /* Joystick Button Ports for Steeringwheel setup */
    public static final int SWJ_LEFT_CLIMB_UP_BUTTON = 8;
    public static final int SWJ_LEFT_CLIMB_DOWN_BUTTON = 12;
    public static final int SWJ_RIGHT_CLIMB_UP_BUTTON = 7;
    public static final int SWJ_RIGHT_CLIMB_DOWN_BUTTON = 11;

    
    /* Numbers */
    public static final int CONTROL_PANEL_MIN_ROTATIONS_IN_TICKS = 20000;
    public static final int TICKS_THREE_ROTATIONS = 20000;
    public static final int ROTATION_CONTROL_SAFETY_TICKS = 1000;

    public static double GRIPPER_MOTOR_SPEED_FORWARD = -0.9;
    public static double GRIPPER_MOTOR_SPEED_REVERSE = 0.5;
    public static double TUNNEL_MOTOR_SPEED_FORWARD = 0.3;
    public static double TUNNEL_MOTOR_SPEED_REVERSE = 0.3;
    public static double TUNNEL_MOTOR_SPEED_FEEDER = 0.7
    ;
    public static double THROWER_MOTOR_REVERSE_SPEED = 0.3;
    public static double THROWER_MOTOR_LOWER_SHAFT_STANDARD_SPEED = -0.48;
    public static double THROWER_MOTOR_UPPER_SHAFT_STANDARD_SPEED = 1.0;
    public static double CONTROL_PANEL_TURN_SPEED = 0.3;
    public static double CONTROL_PANEL_MANUAL_TURN_SPEED = 0.15;
    public static double CLIMBING_MOTOR_SPEED = 0.5;
    public static double MANUAL_CLIMB_SPEED = 0.3;

    public static final double CLIMBER_LEVEL_SAFETY_TICKS = 10;

    public static final double CLIMB_MAX_HEIGHT_TICKS = 95.5;
    public static final double CLIMB_HOVER_HEIGHT = 80;

    public static final double WHEEL_CIRCUMFERENCE = 0.4787787204060999;  // IN METER
    public static final double GEARBOX_TRANSLATION = 10.71;

    /* PID Constants */
    public static final double TURN_KP = 0.17;
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.0;
    public static final double AIM_TOLERANCE = 0.0; // in meter

}
