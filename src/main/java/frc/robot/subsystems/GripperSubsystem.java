/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Motors;

public class GripperSubsystem extends SubsystemBase {

  private final DoubleSolenoid gripperSolenoid = new DoubleSolenoid(Constants.PCM_CAN_ID,
      Constants.GRIPPER_SOLENOID_EXTEND_ID, Constants.GRIPPER_SOLENOID_RETRACT_ID);

  private boolean cancel = false;

  /**
   * Creates a new GripperSubsystem.
   */
  public GripperSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendGripper() {
    gripperSolenoid.set(Value.kForward);
  }

  public void retractGripper() {
    gripperSolenoid.set(Value.kReverse);
  }

  public void closeGripper() {
    gripperSolenoid.close();
  }

  public void setBackwards() {
    Motors.gripper_motor.set(ControlMode.PercentOutput, -Constants.GRIPPER_MOTOR_SPEED);
  }

  public void setForward() {
    Motors.gripper_motor.set(ControlMode.PercentOutput, Constants.GRIPPER_MOTOR_SPEED);
  }

  public void stopGripper() {
    Motors.gripper_motor.stopMotor();
  }

  public void setCancel(boolean cancel) {
    this.cancel = cancel;
  }

  public boolean getCancel() {
    return cancel;
  }

  public boolean getInsideReed() {
    return Motors.gripper_motor.isRevLimitSwitchClosed() == 1;
  }

  public boolean isTurningForward() {
    if(Motors.gripper_motor.get() > 0) {
      return true;
    } else if(Motors.gripper_motor.get() < 0) {
      return false;
    } else {
      throw new IllegalArgumentException("Motor isnt turning");
    }
  }
}