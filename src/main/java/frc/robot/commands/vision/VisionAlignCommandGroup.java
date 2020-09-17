/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands;
import frc.robot.commands.drive.ResetPose;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class VisionAlignCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new VisionAlignCommandGroup.
   */
  public VisionAlignCommandGroup(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new ResetPose(driveSubsystem), new SwitchVisionLightCommand(visionSubsystem), new WaitCommand(0.5), new GenerateTrajectory(visionSubsystem), new WaitCommand(1), new SwitchVisionLightCommand(visionSubsystem));
  }
}