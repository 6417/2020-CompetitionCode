/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups.thrower;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.gripper.GripperForward;
import frc.robot.commands.gripper.GripperRetract;
import frc.robot.commands.gripper.GripperStop;
import frc.robot.commands.thrower.ThrowerExtrudeCommand;
import frc.robot.commands.thrower.ThrowerExtrudeFastCommand;
import frc.robot.commands.tunnel.TunnelSouthCommand;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.subsystems.TunnelSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ThrowerFastCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new ThrowerCommandGroup.
   */
  public ThrowerFastCommandGroup(GripperSubsystem gripperSubsystem, ThrowerSubsystem throwerSubsystem, TunnelSubsystem tunnelSubsystem) {
    super(new GripperStop(gripperSubsystem), new ThrowerExtrudeFastCommand(throwerSubsystem), new TunnelSouthCommand(tunnelSubsystem), new WaitCommand(1), new ThrowerSupplyCommandGroup(tunnelSubsystem, throwerSubsystem), new WaitCommand(0.5), new GripperForward(gripperSubsystem), new GripperRetract(gripperSubsystem), new WaitCommand(0.4), new GripperStop(gripperSubsystem));
  }
}
