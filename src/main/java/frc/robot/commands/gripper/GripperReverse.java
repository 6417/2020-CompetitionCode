/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperReverse extends CommandBase {

  private GripperSubsystem m_subsystem;

  /**
   * Creates a new GripperReverse.
   */
  public GripperReverse(GripperSubsystem subsytem) {
    m_subsystem = subsytem;

    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_subsystem.getInsideReed()) {
      m_subsystem.setCancel(true);
    }
    System.out.println("Gripper Reverse Command initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_subsystem.getCancel() == true) {
      cancel();
    } else {
      m_subsystem.setBackwards();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setCancel(false);
    m_subsystem.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
