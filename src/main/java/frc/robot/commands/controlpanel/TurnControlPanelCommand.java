/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.ColorDetected;

public class TurnControlPanelCommand extends CommandBase {

  private boolean positionResetted;

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
    if(m_subsystem.getBottomReed() || m_subsystem.getFrontReed()) {
      m_subsystem.setCancel(true);
      System.out.println("CP Command Canceled " + m_subsystem.getCancel());
    }
    m_subsystem.resetSensorPos();
    positionResetted = false;
    System.out.println("resetted sensor position");
    m_subsystem.resetColorChanges();
    if(m_subsystem.setStartColorID() == false) {
      m_subsystem.setCancel(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_subsystem.getCancel() == true) {
      cancel();
    } else {
      if(m_subsystem.getSensorPos() < 2000) {
        positionResetted = true;
      }
      m_subsystem.decideMode();
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("End initialized");
    m_subsystem.stopTurn();
    m_subsystem.setCancel(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(positionResetted == true) {
      System.out.println("Encoder Value in is finished " + m_subsystem.getSensorPos());
      return m_subsystem.getFinished();
    } else {
      return false;
    }

  }

}
