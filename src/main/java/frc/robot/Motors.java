/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import ch.team6417.motorcontroller.FridoCANSparkMax;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

/**
 * Add your docs here.
 */
public class Motors {

    /* Motor declarations */
    public static FridoCANSparkMax drive_motor_front_right;
    public static FridoCANSparkMax drive_motor_front_left;
    public static FridoCANSparkMax drive_motor_back_right;
    public static FridoCANSparkMax drive_motor_back_left;

    public static WPI_TalonSRX talon_drive_motor_front_right;
    public static WPI_TalonSRX talon_drive_motor_front_left;
    public static WPI_TalonSRX talon_drive_motor_back_right;
    public static WPI_TalonSRX talon_drive_motor_back_left;

    public static SpeedControllerGroup leftMotors;
    public static SpeedControllerGroup rightMotors;

    public static WPI_TalonSRX control_panel_motor;

    public static WPI_TalonSRX gripper_motor;

    public static WPI_TalonSRX tunnel_motor;

    public static FridoCANSparkMax thrower_motor_lower_shaft;
    public static FridoCANSparkMax thrower_motor_upper_shaft_left;    
    public static FridoCANSparkMax thrower_motor_upper_shaft_right;

    public static CANEncoder thrower_encoder_left;
    public static CANEncoder thrower_encoder_right;

    public static CANPIDController throwerPIDController;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    public static FridoCANSparkMax climber_motor_left;
    public static FridoCANSparkMax climber_motor_right;

    public static CANPIDController climberPIDLeft;
    public static CANPIDController climberPIDRight;

    private double kPclimber, kIclimber, kDclimber, kIzclimber, kFFclimber, kMaxOutputclimber, kMinOutputclimber, maxRPMclimber;

    public static CANEncoder climber_encoder_right;
    public static CANEncoder climber_encoder_left;

    public static CANDigitalInput left_limit;
    public static CANDigitalInput right_limit;

    private PowerDistributionPanel pdp = new PowerDistributionPanel();

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
            if(Constants.TEST_ROBOT) {
                talon_drive_motor_front_right = new WPI_TalonSRX(7);
                talon_drive_motor_front_left = new WPI_TalonSRX(6);
                talon_drive_motor_back_right = new WPI_TalonSRX(4);
                talon_drive_motor_back_left = new WPI_TalonSRX(5);
    
                talon_drive_motor_front_right.configFactoryDefault();
                talon_drive_motor_front_left.configFactoryDefault();
                talon_drive_motor_back_right.configFactoryDefault();
                talon_drive_motor_back_left.configFactoryDefault();    
                
                talon_drive_motor_front_right.setNeutralMode(NeutralMode.Brake);
                talon_drive_motor_front_left.setNeutralMode(NeutralMode.Brake);
                talon_drive_motor_back_right.setNeutralMode(NeutralMode.Brake);
                talon_drive_motor_back_left.setNeutralMode(NeutralMode.Brake);    
    
                talon_drive_motor_back_right.follow(talon_drive_motor_front_right);
                talon_drive_motor_back_left.follow(talon_drive_motor_front_left);
                talon_drive_motor_back_right.setInverted(false);
                talon_drive_motor_back_left.setInverted(false);
    
                leftMotors = new SpeedControllerGroup(talon_drive_motor_front_left, talon_drive_motor_back_left);
                rightMotors = new SpeedControllerGroup(talon_drive_motor_front_right, talon_drive_motor_back_right);
            } else {
                drive_motor_front_right = new FridoCANSparkMax(Constants.MOTOR_DRIVE_FRONT_RIGHT_ID, MotorType.kBrushless);
                drive_motor_front_left = new FridoCANSparkMax(Constants.MOTOR_DRIVE_FRONT_LEFT_ID, MotorType.kBrushless);
                drive_motor_back_right = new FridoCANSparkMax(Constants.MOTOR_DRIVE_BACK_RIGHT_ID, MotorType.kBrushless);
                drive_motor_back_left = new FridoCANSparkMax(Constants.MOTOR_DRIVE_BACK_LEFT_ID, MotorType.kBrushless);

                drive_motor_front_right.restoreFactoryDefaults();
                drive_motor_front_left.restoreFactoryDefaults();
                drive_motor_back_right.restoreFactoryDefaults();
                drive_motor_back_left.restoreFactoryDefaults();   
                
                drive_motor_front_right.setIdleMode(IdleMode.kBrake);
                drive_motor_front_left.setIdleMode(IdleMode.kBrake);
                drive_motor_back_right.setIdleMode(IdleMode.kBrake);
                drive_motor_back_left.setIdleMode(IdleMode.kBrake); 

                drive_motor_back_right.follow(drive_motor_front_right, false);
                drive_motor_back_left.follow(drive_motor_front_left, false);

                leftMotors = new SpeedControllerGroup(drive_motor_front_left, drive_motor_back_left);
                rightMotors = new SpeedControllerGroup(drive_motor_front_right, drive_motor_back_right);
            
           }
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

            thrower_motor_lower_shaft = new FridoCANSparkMax(Constants.MOTOR_LOWER_THROWER_SHAFT_ID, MotorType.kBrushless);
            thrower_motor_upper_shaft_left = new FridoCANSparkMax(Constants.MOTOR_UPPER_THROWER_SHAFT_LEFT_ID, MotorType.kBrushless);
            thrower_motor_upper_shaft_right = new FridoCANSparkMax(Constants.MOTOR_UPPER_THROWER_SHAFT_RIGHT_ID, MotorType.kBrushless);
            
            thrower_motor_lower_shaft.restoreFactoryDefaults();
            thrower_motor_upper_shaft_left.restoreFactoryDefaults();
            thrower_motor_upper_shaft_right.restoreFactoryDefaults();

            thrower_motor_upper_shaft_left.follow(thrower_motor_upper_shaft_right, true);

            thrower_motor_upper_shaft_left.setIdleMode(IdleMode.kBrake);
            thrower_motor_upper_shaft_right.setIdleMode(IdleMode.kBrake);

            thrower_encoder_left = thrower_motor_upper_shaft_left.getEncoder();
            thrower_encoder_right = thrower_motor_upper_shaft_right.getEncoder();
         
            throwerPIDController = thrower_motor_upper_shaft_right.getPIDController();

            // PID coefficients
            kP = 0.00004;
            kI = 0.000001;
            kD = 0.0; 
            kIz = 50.0; 
            kFF = 0.0; 
            kMaxOutput = 1; 
            kMinOutput = -1;
            maxRPM = 5700;

            // set PID coefficients
            throwerPIDController.setP(kP);
            throwerPIDController.setI(kI);
            throwerPIDController.setD(kD);
            throwerPIDController.setIZone(kIz);
            throwerPIDController.setFF(kFF);
            throwerPIDController.setOutputRange(kMinOutput, kMaxOutput);

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

            climber_motor_left = new FridoCANSparkMax(Constants.MOTOR_CLIMBER_LEFT_ID, MotorType.kBrushless);
            climber_motor_right = new FridoCANSparkMax(Constants.MOTOR_CLIMBER_RIGHT_ID, MotorType.kBrushless);

            climber_motor_left.restoreFactoryDefaults();
            climber_motor_right.restoreFactoryDefaults();

            climber_motor_left.setIdleMode(IdleMode.kBrake);
            climber_motor_right.setIdleMode(IdleMode.kBrake);

            climberPIDLeft = climber_motor_left.getPIDController();
            climberPIDRight = climber_motor_right.getPIDController();

            climber_encoder_left = climber_motor_left.getEncoder();
            climber_encoder_right = climber_motor_right.getEncoder();

            // PID coefficients
            kPclimber = 0.0002; 
            kIclimber = 0;
            kDclimber = 0; 
            kIzclimber = 0; 
            kFFclimber = 0.000; 
            kMaxOutputclimber = 0.7; 
            kMinOutputclimber = -0.7;
            maxRPMclimber = 5700;

            // set PID coefficients
            climberPIDLeft.setP(kPclimber);
            climberPIDLeft.setI(kIclimber);
            climberPIDLeft.setD(kDclimber);
            climberPIDLeft.setIZone(kIzclimber);
            climberPIDLeft.setFF(kFFclimber);
            climberPIDLeft.setOutputRange(kMinOutputclimber, kMaxOutputclimber);

            // set PID coefficients
            climberPIDRight.setP(kPclimber);
            climberPIDRight.setI(kIclimber);
            climberPIDRight.setD(kDclimber);
            climberPIDRight.setIZone(kIzclimber);
            climberPIDRight.setFF(kFFclimber);
            climberPIDRight.setOutputRange(kMinOutputclimber, kMaxOutputclimber);


            //Config Limits
            left_limit = climber_motor_left.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            right_limit = climber_motor_right.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

            left_limit.enableLimitSwitch(false);
            right_limit.enableLimitSwitch(false);
        }

    }

    public void disableAll() {
        if(Constants.IS_DRIVE_SUBSYSTEM_IN_USE) {
            leftMotors.stopMotor();
            rightMotors.stopMotor();
        }

        if(Constants.IS_GRIPPER_SUBSYSTEM_IN_USE) {
            gripper_motor.stopMotor();
        }

        if(Constants.IS_TUNNEL_SUBSYSTEM_IN_USE) {
            tunnel_motor.stopMotor();
        }

        if(Constants.IS_CLIMBING_SUBSYSTEM_IN_USE) {
            climber_motor_left.stopMotor();
            climber_motor_right.stopMotor();
        }

        if(Constants.IS_CONTORL_PANEL_SUBSYSTEM_IN_USE) {
            control_panel_motor.stopMotor();
        }

        if(Constants.IS_THROWER_SUBSYSTEM_IN_USE) {
            thrower_motor_upper_shaft_right.stopMotor();
//            thrower_motor_upper_shaft_left.stopMotor();
            thrower_motor_lower_shaft.stopMotor();
        }

    }
    
}
