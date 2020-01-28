/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.thrower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ThrowerSubsystem;

public class ThrowerRevertCommand extends CommandBase {

  private ThrowerSubsystem m_subsystem;

  /**
   * Creates a new ThrowerRevertCommand.
   */
  public ThrowerRevertCommand(ThrowerSubsystem subsystem) {
    m_subsystem = subsystem;
    
    addRequirements(m_subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.turnReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopThrower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
