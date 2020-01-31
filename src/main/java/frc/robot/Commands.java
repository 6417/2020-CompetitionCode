/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.controlpanel.ReadControlPanelFMSData;
import frc.robot.commands.controlpanel.TurnControlPanelCommand;
import frc.robot.commands.gripper.GripperForward;
import frc.robot.commands.gripper.GripperReverse;
import frc.robot.commands.groups.ControlPanelExtendCommandGroup;
import frc.robot.commands.groups.ControlPanelRetractCommandGroup;
import frc.robot.commands.groups.ballflow.FlowForwardRace;
import frc.robot.commands.groups.ballflow.FlowReverseRace;
import frc.robot.commands.groups.ballflow.FlowStopCommandGroup;
import frc.robot.commands.groups.thrower.ThrowerCommandGroup;
import frc.robot.commands.groups.thrower.ThrowerStopCommandGroup;
import frc.robot.commands.vision.ReadVisionDataCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.subsystems.TunnelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Add your docs here.
 */
public class Commands {

    public Commands() {
        initialize();
    }

    // The robot's subsystems and commands are defined here...
    public ControlPanelSubsystem controlPanelSubsystem;

    protected TurnControlPanelCommand turnControlPanelCommand;

    protected ConditionalCommand contorlPanelConditionalCommand;

    public GripperSubsystem gripperSubsystem;

    protected ConditionalCommand gripperSoloTurnConditionalCommand;

    public TunnelSubsystem tunnelSubsystem;

    public ThrowerSubsystem throwerSubsystem;

    protected ConditionalCommand throwerCommandGroup;

    protected ConditionalCommand flowForwardConditionalCommand;
    protected ConditionalCommand flowReverseConditionalCommand;

    public VisionSubsystem visionSubsystem;

    public DriveSubsystem driveSubsystem;

    private void initialize() {

        configControlPanelCommands();

        configGripperCommands();

        configTunnelCommands();

        configDriveCommands();

        configThrowerCommands();

        configFlowCommands();

        configVisionCommands();

    }

    private void configControlPanelCommands() {

        if (Constants.IS_CONTORL_PANEL_SUBSYSTEM_IN_USE) {

            controlPanelSubsystem = new ControlPanelSubsystem();
            controlPanelSubsystem.setDefaultCommand(new ReadControlPanelFMSData(controlPanelSubsystem));

            contorlPanelConditionalCommand = new ConditionalCommand(
                new ControlPanelExtendCommandGroup(controlPanelSubsystem),
                new ControlPanelRetractCommandGroup(controlPanelSubsystem),
                controlPanelSubsystem::getBottomReed);

            turnControlPanelCommand = new TurnControlPanelCommand(controlPanelSubsystem);

        }

    }

    private void configGripperCommands() {

        if (Constants.IS_GRIPPER_SUBSYSTEM_IN_USE) {

            gripperSubsystem = new GripperSubsystem();

            gripperSoloTurnConditionalCommand = new ConditionalCommand(new GripperReverse(gripperSubsystem), new GripperForward(gripperSubsystem),
                    gripperSubsystem::isTurningForward);

        }

    }

    private void configTunnelCommands() {

        if (Constants.IS_TUNNEL_SUBSYSTEM_IN_USE) {

            tunnelSubsystem = new TunnelSubsystem();

        }

    }

    private void configDriveCommands() {

        if(Constants.IS_DRIVE_SUBSYSTEM_IN_USE) {

            driveSubsystem = new DriveSubsystem();

        }

    }

    private void configThrowerCommands() {

        if (Constants.IS_THROWER_SUBSYSTEM_IN_USE && Constants.IS_TUNNEL_SUBSYSTEM_IN_USE) {

            throwerSubsystem = new ThrowerSubsystem();

            throwerCommandGroup = new ConditionalCommand(new ThrowerStopCommandGroup(throwerSubsystem, tunnelSubsystem), new ThrowerCommandGroup(throwerSubsystem, tunnelSubsystem), throwerSubsystem::isrunning);

        }
        

    }

    private void configFlowCommands() {

        if (Constants.IS_TUNNEL_SUBSYSTEM_IN_USE && Constants.IS_GRIPPER_SUBSYSTEM_IN_USE
                && Constants.IS_THROWER_SUBSYSTEM_IN_USE) {

            flowForwardConditionalCommand = new ConditionalCommand(
                    new FlowForwardRace(gripperSubsystem, tunnelSubsystem),
                    new FlowStopCommandGroup(gripperSubsystem, tunnelSubsystem, throwerSubsystem),
                    gripperSubsystem::getInsideReed);
            flowReverseConditionalCommand = new ConditionalCommand(
                    new FlowReverseRace(gripperSubsystem, tunnelSubsystem, throwerSubsystem),
                    new FlowStopCommandGroup(gripperSubsystem, tunnelSubsystem, throwerSubsystem),
                    gripperSubsystem::getInsideReed);

        }

    }

    private void configVisionCommands() {

        if (Constants.IS_VISION_SUBSYSTEM_IN_USE) {

            visionSubsystem = new VisionSubsystem();
            visionSubsystem.setDefaultCommand(new ReadVisionDataCommand(visionSubsystem));

        }

    }

}
