/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class Motors {

    /* Motor declarations */
    public static WPI_TalonSRX drive_motor_front_right;
    public static WPI_TalonSRX drive_motor_front_left;
    public static WPI_TalonSRX drive_motor_back_right;
    public static WPI_TalonSRX drive_motor_back_left;

    public static WPI_TalonSRX control_panel_motor;

    public static WPI_TalonSRX gripper_motor;

    public static WPI_TalonSRX tunnel_motor;

    public static CANSparkMax thrower_motor_lower_shaft;
    public static CANSparkMax thrower_motor_upper_shaft_left;    
    public static CANSparkMax thrower_motor_upper_shaft_right;

    public static CANSparkMax climber_motor_left;
    public static CANSparkMax climber_motor_right;

    public static CANEncoder climber_encoder_right;
    public static CANEncoder climber_encoder_left;

    public Motors() {

        configDriveMotors();

        configContorlPanelMotors();

        configGripperMotors();

        configThrowerMotors();

        configTunnelMotors();

        configClimberMotors();

    }    
    
    private void configDriveMotors() {
        
        if(Constants.IS_DRIVE_SUBSYSTEM_IN_USE) {

            drive_motor_front_right = new WPI_TalonSRX(Constants.MOTOR_DRIVE_FRONT_RIGHT_ID);
            drive_motor_front_left = new WPI_TalonSRX(Constants.MOTOR_DRIVE_FRONT_LEFT_ID);
            drive_motor_back_right = new WPI_TalonSRX(Constants.MOTOR_DRIVE_BACK_RIGHT_ID);
            drive_motor_back_left = new WPI_TalonSRX(Constants.MOTOR_DRIVE_BACK_LEFT_ID);

            drive_motor_front_right.configFactoryDefault();
            drive_motor_front_left.configFactoryDefault();
            drive_motor_back_right.configFactoryDefault();
            drive_motor_back_left.configFactoryDefault();            

            drive_motor_back_right.follow(drive_motor_front_right);
            drive_motor_back_left.follow(drive_motor_front_left);


        }

    }

    private void configContorlPanelMotors() {

        if(Constants.IS_CONTORL_PANEL_SUBSYSTEM_IN_USE) {

            control_panel_motor = new WPI_TalonSRX(Constants.MOTOR_CONTROL_PANEL_ID);
            
            control_panel_motor.configFactoryDefault();

            control_panel_motor.setNeutralMode(NeutralMode.Brake);
        }

    }

    private void configGripperMotors() {

        if(Constants.IS_GRIPPER_SUBSYSTEM_IN_USE) {

            gripper_motor = new WPI_TalonSRX(Constants.MOTOR_GRIPPER_ID);

            gripper_motor.configFactoryDefault();

        }

    }

    private void configThrowerMotors() {

        if(Constants.IS_THROWER_SUBSYSTEM_IN_USE) {

            thrower_motor_lower_shaft = new CANSparkMax(Constants.MOTOR_LOWER_THROWER_SHAFT_ID, MotorType.kBrushless);
            thrower_motor_upper_shaft_left = new CANSparkMax(Constants.MOTOR_UPPER_THROWER_SHAFT_LEFT_ID, MotorType.kBrushless);
            thrower_motor_upper_shaft_right = new CANSparkMax(Constants.MOTOR_UPPER_THROWER_SHAFT_RIGHT_ID, MotorType.kBrushless);
            
            thrower_motor_lower_shaft.restoreFactoryDefaults();
            thrower_motor_upper_shaft_left.restoreFactoryDefaults();
            thrower_motor_upper_shaft_right.restoreFactoryDefaults();

            thrower_motor_upper_shaft_left.follow(thrower_motor_upper_shaft_right, true);
                                                                       
        }

    }

    private void configTunnelMotors() {

        if(Constants.IS_TUNNEL_SUBSYSTEM_IN_USE) {

            tunnel_motor = new WPI_TalonSRX(Constants.MOTOR_TUNNEL_ID);

            tunnel_motor.configFactoryDefault();

        }

    }

    private void configClimberMotors() {

        if(Constants.IS_CLIMBING_SUBSYSTEM_IN_USE) {

            climber_motor_left = new CANSparkMax(Constants.MOTOR_CLIMBER_LEFT_ID, MotorType.kBrushless);
            climber_motor_right = new CANSparkMax(Constants.MOTOR_CLIMBER_RIGHT_ID, MotorType.kBrushless);

            climber_motor_left.restoreFactoryDefaults();
            climber_motor_right.restoreFactoryDefaults();

            climber_motor_left.follow(climber_motor_right, true);

            climber_encoder_left = climber_motor_left.getEncoder();
            climber_encoder_right = climber_motor_right.getEncoder();

        }

    }
    
}
