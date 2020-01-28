/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.gripper.GripperRetract;
import frc.robot.commands.gripper.GripperStop;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class GripperRetractCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new GripperRetractCommandGroup.
   */
  public GripperRetractCommandGroup(GripperSubsystem subsystem) {
    super(new GripperStop(subsystem), new WaitCommand(1), new GripperRetract(subsystem));
  }
}
