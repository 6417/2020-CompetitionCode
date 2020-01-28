/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class TurnControlPanelCommand extends CommandBase {

  private final ControlPanelSubsystem m_subsystem;
//  private boolean toCancel = false;
  /**
   * Creates a new TurnControlPanelCommand.
   */
  public TurnControlPanelCommand(ControlPanelSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_subsystem.getBottomReed()) {
      m_subsystem.setCancel(true);
    }
    m_subsystem.resetSensorPos();
    System.out.println("resetted sensor position");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_subsystem.getCancel() == true) {
      cancel();
    } else {
      m_subsystem.decideMode();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopTurn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.getFinished();
  }

}
