/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups.ballflow;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.groups.GripperForwardCommandGroup;
import frc.robot.commands.tunnel.TunnelNorthCommand;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.subsystems.TunnelSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FlowForwardRace extends ParallelRaceGroup {
  /**
   * Creates a new FlowForwardRace.
   */
  public FlowForwardRace(GripperSubsystem gripperSubsystem, TunnelSubsystem tunnelSubsystem, ThrowerSubsystem throwerSubsystem) {
    super(new GripperForwardCommandGroup(gripperSubsystem), new TunnelNorthCommand(tunnelSubsystem));
    System.out.println("Flow Forward Race activated");
  }
}
