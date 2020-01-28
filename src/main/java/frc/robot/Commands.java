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
import frc.robot.commands.thrower.ThrowerExtrudeCommand;
import frc.robot.commands.vision.ReadVisionDataCommand;
import frc.robot.subsystems.ControlPanelSubsystem;
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

    private ControlPanelExtendCommandGroup controlPanelExtend;
    private ControlPanelRetractCommandGroup controlPanelRetract;

    private ReadControlPanelFMSData readControlPanelCommand;

    protected TurnControlPanelCommand turnControlPanelCommand;

    protected ConditionalCommand contorlPanelConditionalCommand;



    public GripperSubsystem gripperSubsystem;

    private GripperForward gripperForwardCommand;
    private GripperReverse gripperReverseCommand;

    protected ConditionalCommand gripperSoloTurnConditionalCommand;



    public TunnelSubsystem tunnelSubsystem;

    public ThrowerSubsystem throwerSubsystem;

    protected ThrowerExtrudeCommand throwerExtrude;

    private FlowForwardRace flowForward;
    private FlowReverseRace flowReverse;
    private FlowStopCommandGroup flowStop;

    protected ConditionalCommand flowForwardConditionalCommand;
    protected ConditionalCommand flowReverseConditionalCommand;

    public VisionSubsystem visionSubsystem;
    private ReadVisionDataCommand readVisionData;

    

    private void initialize() {

        configControlPanelCommands();

        configGripperCommands();

        configTunnelCommands();

        configThrowerCommands();

        configFlowCommands();

        configVisionCommands();

    }

    private void configControlPanelCommands() {

        if(Constants.IS_CONTORL_PANEL_SUBSYSTEM_IN_USE) {

            controlPanelSubsystem = new ControlPanelSubsystem();
            readControlPanelCommand = new ReadControlPanelFMSData(controlPanelSubsystem);
            controlPanelSubsystem.setDefaultCommand(readControlPanelCommand);
    
            controlPanelExtend = new ControlPanelExtendCommandGroup(controlPanelSubsystem);
            controlPanelRetract = new ControlPanelRetractCommandGroup(controlPanelSubsystem);
    
            contorlPanelConditionalCommand = new ConditionalCommand(controlPanelExtend, controlPanelRetract, controlPanelSubsystem::getBottomReed);
    
            turnControlPanelCommand = new TurnControlPanelCommand(controlPanelSubsystem);

        }

    }

    private void configGripperCommands() {

        if(Constants.IS_GRIPPER_SUBSYSTEM_IN_USE) {

            gripperSubsystem = new GripperSubsystem();

            gripperForwardCommand = new GripperForward(gripperSubsystem);
            gripperReverseCommand = new GripperReverse(gripperSubsystem);
    
            gripperSoloTurnConditionalCommand = new ConditionalCommand(gripperReverseCommand, gripperForwardCommand, gripperSubsystem::isTurningForward);

        }

    }

    private void configTunnelCommands() {

        if(Constants.IS_TUNNEL_SUBSYSTEM_IN_USE) {

            tunnelSubsystem = new TunnelSubsystem();

        }

    }

    private void configThrowerCommands() {

        if(Constants.IS_THROWER_SUBSYSTEM_IN_USE) {

            throwerSubsystem = new ThrowerSubsystem();

            throwerExtrude = new ThrowerExtrudeCommand(throwerSubsystem);

        }

    }

    private void configFlowCommands() {

        if(Constants.IS_TUNNEL_SUBSYSTEM_IN_USE && Constants.IS_GRIPPER_SUBSYSTEM_IN_USE && Constants.IS_THROWER_SUBSYSTEM_IN_USE) {
            
            flowForward = new FlowForwardRace(gripperSubsystem, tunnelSubsystem);
            flowReverse = new FlowReverseRace(gripperSubsystem, tunnelSubsystem, throwerSubsystem);
            flowStop = new FlowStopCommandGroup(gripperSubsystem, tunnelSubsystem, throwerSubsystem);

            flowForwardConditionalCommand = new ConditionalCommand(flowForward, flowStop, gripperSubsystem::getInsideReed);
            flowReverseConditionalCommand = new ConditionalCommand(flowReverse, flowStop, gripperSubsystem::getInsideReed);

        }

    }

    private void configVisionCommands() {

        if(Constants.IS_VISION_SUBSYSTEM_IN_USE) {
            
            visionSubsystem = new VisionSubsystem();
            visionSubsystem.setDefaultCommand(readVisionData);

        }

    }

}
