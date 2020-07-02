/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.gripper.GripperProtectorExtend;
import frc.robot.commands.groups.ballflow.FlowForwardRace;
import frc.robot.commands.groups.ballflow.FlowStopCommandGroup;
import frc.robot.commands.groups.thrower.ThrowerCommandGroup;
import frc.robot.commands.groups.thrower.ThrowerStopCommandGroup;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.subsystems.TunnelSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousSystemControlCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new AutonomousSystemCommandGroup.
   */
  public AutonomousSystemControlCommandGroup(GripperSubsystem gripperSubsystem, ThrowerSubsystem throwerSubsystem, TunnelSubsystem tunnelSubsystem) {
       
    //This section is for a six ball autonomous with shooting two times
    super(new GripperProtectorExtend(gripperSubsystem), new WaitCommand(0), new ThrowerCommandGroup(gripperSubsystem, throwerSubsystem, tunnelSubsystem),
      new WaitCommand(1.5), new ThrowerStopCommandGroup(throwerSubsystem, tunnelSubsystem), new WaitCommand(0.5),
      new FlowForwardRace(gripperSubsystem, tunnelSubsystem, throwerSubsystem), new WaitCommand(6),
      new FlowStopCommandGroup(gripperSubsystem, tunnelSubsystem, throwerSubsystem), new WaitCommand(4),
      new ThrowerCommandGroup(gripperSubsystem, throwerSubsystem, tunnelSubsystem), new WaitCommand(3),
      new ThrowerStopCommandGroup(throwerSubsystem, tunnelSubsystem));

    //This section is for autonomous shooting before grabing balls the newly grabed balls wont be shot out
    /* super(new GripperProtectorExtend(gripperSubsystem), new WaitCommand(0), new ThrowerCommandGroup(gripperSubsystem, throwerSubsystem, tunnelSubsystem),
      new WaitCommand(1.5), new ThrowerStopCommandGroup(throwerSubsystem, tunnelSubsystem), new WaitCommand(0.5),
      new FlowForwardRace(gripperSubsystem, tunnelSubsystem, throwerSubsystem), new WaitCommand(6),
      new FlowStopCommandGroup(gripperSubsystem, tunnelSubsystem, throwerSubsystem));
    */

    //This section is for the drive from line trajectory or a simple non driving autonomous
    /* super(new GripperProtectorExtend(gripperSubsystem), new WaitCommand(0), new ThrowerCommandGroup(gripperSubsystem, throwerSubsystem, tunnelSubsystem),
      new WaitCommand(1.5), new ThrowerStopCommandGroup(throwerSubsystem, tunnelSubsystem));*/
  }
}
