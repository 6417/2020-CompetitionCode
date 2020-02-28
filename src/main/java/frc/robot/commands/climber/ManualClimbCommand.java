/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Motors;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbType;

public class ManualClimbCommand extends CommandBase {

  private ClimberSubsystem m_subsystem;

  /**
   * Creates a new ManualClimbCommand.
   */
  public ManualClimbCommand(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;

    addRequirements(m_subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.manualLeftClimbDownButton.get()) {
      m_subsystem.climbDownLeft();
    } else if(RobotContainer.manualLeftClimbUpButton.get()) {
      m_subsystem.climbUpLeft();
    } else {
      Motors.climber_motor_left.stopMotor();
    }

    if(RobotContainer.manualRightClimbDownButton.get()) {
      m_subsystem.climbDownRight();
    } else if(RobotContainer.manualRightClimbUpButton.get()) {
      m_subsystem.climbUpRight();
    } else {
      Motors.climber_motor_right.stopMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
