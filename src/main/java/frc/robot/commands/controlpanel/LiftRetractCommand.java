/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.ControlPanelMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LiftRetractCommand extends InstantCommand {

  private final ControlPanelSubsystem m_subsystem;

  public LiftRetractCommand(ControlPanelSubsystem subsystem) {
    m_subsystem = subsystem;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.retractLift();
  }

  @Override
  public void end(boolean interrupted) {
    if(m_subsystem.getSensorPos() >= Constants.CONTROL_PANEL_MIN_ROTATIONS_IN_TICKS) {
      m_subsystem.setMode(ControlPanelMode.POSITION_CONTROL);
    }
  }
}
