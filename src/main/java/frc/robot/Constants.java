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
    public static final boolean STEERING_WHEEL_USAGE = false;

    /* Subsystems in use */
    public static final boolean IS_DRIVE_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_CONTORL_PANEL_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_PNEUMATICS_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_GRIPPER_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_THROWER_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_TUNNEL_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_CLIMBING_SUBSYSTEM_IN_USE = true;
    public static final boolean IS_VISION_SUBSYSTEM_IN_USE = true;

    /* PCM CAN ID */
    public static final int PCM_CAN_ID = 20;

    /* Motor ID */
    public static final int MOTOR_DRIVE_FRONT_RIGHT_ID = 1;
    public static final int MOTOR_DRIVE_FRONT_LEFT_ID = 2;
    public static final int MOTOR_DRIVE_BACK_RIGHT_ID = 3;
    public static final int MOTOR_DRIVE_BACK_LEFT_ID = 4;

    public static final int MOTOR_CONTROL_PANEL_ID = 48;

    public static final int MOTOR_GRIPPER_ID = 12;

    public static final int MOTOR_LOWER_THROWER_SHAFT_ID = 5;
    public static final int MOTOR_UPPER_THROWER_SHAFT_LEFT_ID = 6; 
    public static final int MOTOR_UPPER_THROWER_SHAFT_RIGHT_ID = 7;

    public static final int MOTOR_TUNNEL_ID = 8;

    public static final int MOTOR_CLIMBER_LEFT_ID = 9;
    public static final int MOTOR_CLIMBER_RIGHT_ID = 10;

    /* Solenoid Ports */
    public static final int CONTROL_PANEL_LIFT_SOLENOID_EXTEND_ID = 0;
    public static final int CONTROL_PANEL_LIFT_SOLENOID_RETRACT_ID = 1;
    public static final int CONTROL_PANEL_DAMPING_SOLENOID_EXTEND_ID = 2;
    public static final int CONTROL_PANEL_DAMPING_SOLENOID_RETRACT_ID = 3;

    public static final int GRIPPER_SOLENOID_EXTEND_ID = 4;
    public static final int GRIPPER_SOLENOID_RETRACT_ID = 5;

    /* Sensors */
    public static final I2C.Port CONTROL_PANEL_COLOR_SENSOR_I2C_ID = I2C.Port.kOnboard;

    public static final int CONTROL_PANEL_LIFT_BOTTOM_REED_DI_ID = 0;
    public static final int CONTROL_PANEL_LIFT_TOP_REED_DI_ID = 1;
    public static final int CONTROL_PANEL_DAMPING_BACK_REED_DI_ID = 2;
    public static final int CONTROL_PANEL_DAMPING_FRONT_REED_DI_ID = 3;

    /* Joystick Port */
    public static final int JOYSTICK_STEER_ID = 0;
    public static final int JOYSTICK_DRIVE_ID = 1;

    public static final int SINGLE_JOYSTICK = 0;

    /* Joystick Button Ports for single controller */
    public static final int SJ_CONTROL_PANEL_LIFT_BUTTON_ID = 1;
    public static final int SJ_CONTROL_PANEL_TURN_BUTTON_ID = 2;

    public static final int SJ_FLOW_FORWARD_BUTTON_ID = 3;
    public static final int SJ_FLOW_REVERSE_BUTTON_ID = 4;
    public static final int SJ_GRIPPER_SOLO_TURN_BUTTON_ID = 5;
    public static final int SJ_THROWER_ENABLE_BUTTON_ID = 6;

    /* Joystick Button Ports for Steeringwheel setup */

    
    /* Numbers */
    public static final int CONTROL_PANEL_MIN_ROTATIONS_IN_TICKS = 20000;

    public static final double GRIPPER_MOTOR_SPEED = 0.3;
    public static final double TUNNEL_MOTOR_SPEED = 0.3;
    public static final double THROWER_MOTOR_REVERSE_SPEED = 0.3;
    public static final double THROWER_MOTOR_LOWER_SHAFT_STANDARD_SPEED = 0.48;
    public static final double THROWER_MOTOR_UPPER_SHAFT_STANDARD_SPEED = 1.0;
    public static final double CLIMBING_MOTOR_SPEED = 0.5;

}
