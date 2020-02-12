/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Motors;
import frc.robot.ShuffleBoard;

public class TunnelSubsystem extends SubsystemBase {
  /**
   * Creates a new TunnelSubsystem.
   */
  public TunnelSubsystem() {
    super.addChild("Tunnel Motor", Motors.tunnel_motor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runForward() {
    Motors.tunnel_motor.set(ControlMode.PercentOutput, Constants.TUNNEL_MOTOR_SPEED);
//    Motors.tunnel_motor.set(ControlMode.PercentOutput, ShuffleBoard.tunnelMotor.getDouble(0.0));
  }

  public void runReverse() {
    Motors.tunnel_motor.set(ControlMode.PercentOutput, -Constants.TUNNEL_MOTOR_SPEED);
  }

  public void stopTunnel() {
    Motors.tunnel_motor.stopMotor();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
  }

}
