/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class GenerateTrajectory extends InstantCommand {

  private final VisionSubsystem m_VisionSubsystem;

  public GenerateTrajectory(final VisionSubsystem visionSubsystem) {
    m_VisionSubsystem = visionSubsystem;

    addRequirements(m_VisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Trajectory trajectory = m_VisionSubsystem.generateTrajectory();
    RamseteCommand command = m_VisionSubsystem.generateRamseteCommand(trajectory);
    m_VisionSubsystem.setRamseteCommand(command);
    command.andThen(() -> Commands.driveSubsystem.tankDriveVolts(0, 0)).schedule(false);
  }
}
