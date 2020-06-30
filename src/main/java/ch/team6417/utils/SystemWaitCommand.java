/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.team6417.utils;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SystemWaitCommand extends CommandBase {

  /**
   * Creates a new SystemWaitCommand.
   */
  public SystemWaitCommand() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("first Spline ended");  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
