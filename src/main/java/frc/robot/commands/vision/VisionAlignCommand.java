/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends CommandBase {

  private DriveSubsystem m_driveSubsystem;
  private VisionSubsystem m_visionSubsystem;

  /**
   * Creates a new VisionAlignCommand.
   */
  public VisionAlignCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_driveSubsystem = driveSubsystem;

    addRequirements(m_visionSubsystem, m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Constants.IS_VISION_SUBSYSTEM_IN_USE) {
      return m_visionSubsystem.isAligned();
    } else {
      return true;
    }
  }
}
