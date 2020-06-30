/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;

  /**
   * Creates a new DriveCommand.
   */
  public DriveCommand(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;

    addRequirements(m_driveSubsystem);
  }

  @Override
  public void execute() {
    m_driveSubsystem.arcadeDrive();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
