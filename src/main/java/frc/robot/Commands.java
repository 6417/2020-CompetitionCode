/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.climber.ClimbStopCommand;
import frc.robot.commands.climber.ClimbUPCommand;
import frc.robot.commands.controlpanel.ReadControlPanelFMSData;
import frc.robot.commands.controlpanel.TurnControlPanelCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.groups.ControlPanelExtendCommandGroup;
import frc.robot.commands.groups.ControlPanelRetractCommandGroup;
import frc.robot.commands.groups.ballflow.FlowForwardRace;
import frc.robot.commands.groups.ballflow.FlowReverseRace;
import frc.robot.commands.groups.ballflow.FlowStopCommandGroup;
import frc.robot.commands.groups.thrower.ThrowerCommandGroup;
import frc.robot.commands.groups.thrower.ThrowerStopCommandGroup;
import frc.robot.commands.vision.ReadVisionDataCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
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

    // The robot's subsystems and commands are defined here..
    public static PneumaticsSubsystem pneumaticsSubsystem;
    
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
    protected FlowStopCommandGroup flowStopCommandGroup;

    public VisionSubsystem visionSubsystem;

    public DriveSubsystem driveSubsystem;

    protected DriveCommand driveCommand;

    public ClimberSubsystem climberSubsystem;

    protected ClimbUPCommand climbUPCommand;
    protected ClimbStopCommand climbStopCommand;

    private void initialize() {

        configPneumatics();

        configControlPanelCommands();

        configGripperCommands();

        configTunnelCommands();

        configDriveCommands();

        configThrowerCommands();

        configFlowCommands();

        configVisionCommands();

        configClimbCommands();

    }

    private void configPneumatics() {

        if(Constants.IS_PNEUMATICS_SUBSYSTEM_IN_USE) {

            pneumaticsSubsystem = new PneumaticsSubsystem();

        }

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

//            gripperSoloTurnConditionalCommand = new ConditionalCommand(new GripperReverse(gripperSubsystem), new GripperForward(gripperSubsystem),
//                    gripperSubsystem::isTurningForward);

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

            driveCommand = new DriveCommand(driveSubsystem);

            driveSubsystem.setDefaultCommand(driveCommand);

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
                && Constants.IS_THROWER_SUBSYSTEM_IN_USE && false == false) { //TODO change Statement for Flow commands

            flowStopCommandGroup = new FlowStopCommandGroup(gripperSubsystem, tunnelSubsystem, throwerSubsystem);
            flowForwardConditionalCommand = new ConditionalCommand(
                    new FlowStopCommandGroup(gripperSubsystem, tunnelSubsystem, throwerSubsystem),
                    new FlowForwardRace(gripperSubsystem, tunnelSubsystem, throwerSubsystem),
                    gripperSubsystem::isTurningForward);
            flowReverseConditionalCommand = new ConditionalCommand(
                    new FlowStopCommandGroup(gripperSubsystem, tunnelSubsystem, throwerSubsystem),
                    new FlowReverseRace(gripperSubsystem, tunnelSubsystem, throwerSubsystem),
                    gripperSubsystem::isTurningReverse);

        }

    }

    private void configVisionCommands() {

        if (Constants.IS_VISION_SUBSYSTEM_IN_USE) {

            visionSubsystem = new VisionSubsystem();
            visionSubsystem.setDefaultCommand(new ReadVisionDataCommand(visionSubsystem));

        }

    }

    private void configClimbCommands() {

        if(Constants.IS_CLIMBING_SUBSYSTEM_IN_USE && Constants.IS_DRIVE_SUBSYSTEM_IN_USE) {

            climberSubsystem = new ClimberSubsystem();
            climbUPCommand = new ClimbUPCommand(climberSubsystem, driveSubsystem);
            climbStopCommand = new ClimbStopCommand(climberSubsystem);

        } else if(Constants.IS_CLIMBING_SUBSYSTEM_IN_USE) {

            climberSubsystem = new ClimberSubsystem();
            climbUPCommand = new ClimbUPCommand(climberSubsystem);
            climbStopCommand = new ClimbStopCommand(climberSubsystem);

        }

    }

}
