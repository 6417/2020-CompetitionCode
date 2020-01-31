/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Motors;

public class DriveSubsystem extends SubsystemBase {

  private DifferentialDrive diffdrive;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    diffdrive = new DifferentialDrive(Motors.drive_motor_front_left, Motors.drive_motor_front_right);
    diffdrive.setRightSideInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  



}
