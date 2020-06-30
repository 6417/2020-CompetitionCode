/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import ch.team6417.utils.SystemWaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.ResetPose;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TrajectoryDrive extends SequentialCommandGroup {
  /**
   * Creates a new TrajectoryDrive.
   */
  public TrajectoryDrive(DriveSubsystem driveSubsystem) {
    // super(new ResetPose(driveSubsystem), new WaitCommand(7), new RamsedeCommand(1), new WaitCommand(5), new RamsedeCommand(2));
    super(new ResetPose(driveSubsystem), new WaitCommand(3), new RamsedeCommand(1), new WaitCommand(9), new ResetPose(driveSubsystem), new RamsedeCommand(0), new WaitCommand(7));
  }
}
