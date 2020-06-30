/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {

  private Compressor compressor = new Compressor(Constants.PCM_CAN_ID);

  public enum PneumaticState {
    OFF, FORWARD, REVERSE
  }

  /**
   * Creates a new PneumaticsSubsystem.
   */
  public PneumaticsSubsystem() {
    compressor.setClosedLoopControl(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /* Getters and Setters for the controll of the PCM */
  public boolean isCompressorEnabled() {
    return compressor.enabled();
  } 

  public boolean isClosedLoopControl() {
    return compressor.getClosedLoopControl();
  }

  public boolean getPreasureSwitchValue() {
    return compressor.getPressureSwitchValue();
  }
     
  public double getCompressorAmps() {
    return compressor.getCompressorCurrent();
  }

  public boolean getFaultState() {
    return compressor.getCompressorCurrentTooHighFault();
  }

  public boolean getStickyFaultState() {
    return compressor.getCompressorCurrentTooHighStickyFault();
  }

  public boolean isCompressorStickyConected() {
    return !compressor.getCompressorNotConnectedStickyFault();
  }

  public boolean isCompressorConnected() {
    return !compressor.getCompressorNotConnectedFault();
  }

  public boolean isOutputShortend() {
    return compressor.getCompressorShortedFault();
  }

  public boolean isOutputStickyShortend() {
    return compressor.getCompressorShortedStickyFault();
  }

  public void resetPCM() {
    compressor.clearAllPCMStickyFaults();
  }

  public void startCompressor() {
    compressor.start();
  }

  public void stopCompressor() {
    compressor.stop();
  }

  public void setLoopControl(boolean on) {
    compressor.setClosedLoopControl(on);
  }
}
