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

  private final DoubleSolenoid gripperProtectorSolenoid = new DoubleSolenoid(Constants.PCM_CAN_ID,
      Constants.GRIPPER_PROTECTOR_SOLENOID_EXTEND_ID, Constants.GRIPPER_PROTECTOR_SOLENOID_RETRACT_ID);

  private boolean cancel = false;
  private boolean protectorExtended = false;
  private boolean gripperExtended = false;

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

  public void extendProtector() {
    gripperProtectorSolenoid.set(Value.kForward);
  }

  public void retractGripper() {
    gripperSolenoid.set(Value.kReverse);
  }

  public void retractProtector() {
    gripperProtectorSolenoid.set(Value.kReverse);
  }

  public void closeGripper() {
    gripperSolenoid.close();
  }

  public void closeProtector() {
    gripperProtectorSolenoid.close();
  }

  public void setBackwards() {
    Motors.gripper_motor.set(ControlMode.PercentOutput, Constants.GRIPPER_MOTOR_SPEED_REVERSE);
  }

  public void setForward() {
    Motors.gripper_motor.set(ControlMode.PercentOutput, Constants.GRIPPER_MOTOR_SPEED_FORWARD);
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
//    return Motors.gripper_motor.isRevLimitSwitchClosed() == 0;
    return false;
  }

public void setProtectorExtended(boolean extended) {
  protectorExtended = extended;
}

public void setGripperExtended(boolean extended) {
  gripperExtended = extended;
}

public boolean isProtectorExtended() {
  return protectorExtended;
}

public boolean isGripperExtended() {
  return gripperExtended;
}

  public boolean isTurningForward() {
    if(Motors.gripper_motor.getMotorOutputVoltage() < 0) {
      System.out.println("Gripper Motor returned true");
      return true;
    } else if(Motors.gripper_motor.getMotorOutputVoltage() > 0) {
      System.out.println("Gripper Motor returned false");
      return false;
    } else if(Motors.gripper_motor.getMotorOutputVoltage() == 0) {
      System.out.println("Gripper Motor not turning");
      return false;
    } else {
      System.out.println("Gripper Motor return default");
      return false;
      //      throw new IllegalArgumentException("Motor isnt turning");
    }
  }

  public boolean isTurningReverse() {
    if(Motors.gripper_motor.get() > 0) {
      System.out.println("Gripper Motor returned false");
      return false;
    } else if(Motors.gripper_motor.get() < 0) {
      System.out.println("Gripper Motor returned true");
      return true;
    } else {
      return false;
      //      throw new IllegalArgumentException("Motor isnt turning");
    }
  }
}
