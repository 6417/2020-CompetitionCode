/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.controlpanel.DamperExtendCommand;
import frc.robot.commands.controlpanel.LiftExtendCommand;
import frc.robot.subsystems.ControlPanelSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ControlPanelExtendCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new ControlPanelExtendCommandGroup.
   */
  public ControlPanelExtendCommandGroup(ControlPanelSubsystem subsystem) {
    super(new LiftExtendCommand(subsystem), new DamperExtendCommand(subsystem));
    System.out.println("Extend initialized");
  }
}
