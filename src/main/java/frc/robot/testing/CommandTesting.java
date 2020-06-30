/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.testing;

import java.lang.reflect.Field;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.controlpanel.DamperExtendCommand;
import frc.robot.commands.controlpanel.DamperRetractCommand;
import frc.robot.commands.controlpanel.LiftExtendCommand;
import frc.robot.commands.controlpanel.LiftRetractCommand;
import frc.robot.commands.controlpanel.ReadControlPanelFMSData;
import frc.robot.commands.controlpanel.TurnControlPanelCommand;
import frc.robot.commands.gripper.GripperExtend;
import frc.robot.commands.gripper.GripperForward;
import frc.robot.commands.gripper.GripperRetract;
import frc.robot.commands.gripper.GripperReverse;
import frc.robot.commands.gripper.GripperStop;
import frc.robot.commands.groups.GripperForwardCommandGroup;
import frc.robot.commands.thrower.ThrowerExtrudeCommand;
import frc.robot.commands.thrower.ThrowerRevertCommand;
import frc.robot.commands.thrower.ThrowerStopCommand;
import frc.robot.commands.thrower.ThrowerSupplyCommand;
import frc.robot.commands.tunnel.TunnelNorthCommand;
import frc.robot.commands.tunnel.TunnelSouthCommand;
import frc.robot.commands.tunnel.TunnelStopCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.subsystems.TunnelSubsystem;

/**
 * Add your docs here.
 */
public class CommandTesting {

    private PneumaticsSubsystem pneumaticsSubsystem;
    private ControlPanelSubsystem controlPanelSubsystem;
    private GripperSubsystem gripperSubsystem;
    private DriveSubsystem driveSubsystem;
    private ThrowerSubsystem throwerSubsystem;
    private TunnelSubsystem tunnelSubsystem;
    private ClimberSubsystem climberSubsystem;

    private DamperExtendCommand damperExtendCommand;
    private DamperRetractCommand damperRetractCommand;
    private LiftExtendCommand liftExtendCommand;
    private LiftRetractCommand liftRetractCommand;
    private ReadControlPanelFMSData readControlPanelFMSData;
    private TurnControlPanelCommand turnControlPanelCommand;

    private GripperExtend gripperExtend;
    private GripperForward gripperForward;
    private GripperRetract gripperRetract;
    private GripperReverse gripperReverse;
    private GripperStop gripperStop;

    private ThrowerExtrudeCommand throwerExtrudeCommand;
    private ThrowerRevertCommand throwerRevertCommand;
    private ThrowerStopCommand ThrowerStopCommand;
    private ThrowerSupplyCommand throwerSupplyCommand;

    private TunnelNorthCommand tunnelNorthCommand;
    private TunnelSouthCommand tunnelSouthCommand;
    private TunnelStopCommand tunnelStopCommand;

    private final ArrayList<Command> commandObjects = new ArrayList<>(18);
    private final ArrayList<Command> initializedCommands = new ArrayList<>();
    private final ArrayList<ArrayList<Command> > possibleCommands = new ArrayList<>(10);

    private final Class c = CommandTesting.class;
    private Field f[];

    public CommandTesting() {

    }

    private void initializeGripper() {
        if (Constants.IS_GRIPPER_SUBSYSTEM_IN_USE) {
            if (TestModeValues.GRIPPER_PNEUMATICS_ENABLED == false && TestModeValues.GRIPPER_MOTOR_ENABLED == true) {

            } else if (TestModeValues.GRIPPER_PNEUMATICS_ENABLED == true
                    && TestModeValues.GRIPPER_MOTOR_ENABLED == false) {

            } else if (TestModeValues.GRIPPER_PNEUMATICS_ENABLED == true
                    && TestModeValues.GRIPPER_MOTOR_ENABLED == true) {

            } else {

            }
        } else {

        }
    }

    private void initializeControlPanel() {
        if (Constants.IS_CONTORL_PANEL_SUBSYSTEM_IN_USE) {
            if (TestModeValues.CONTROL_PANEL_PNEUMATICS_ENABLED == true
                    && TestModeValues.CONTROL_PANEL_MOTOR_ENABLED == false) {

            } else if (TestModeValues.CONTROL_PANEL_PNEUMATICS_ENABLED == false
                    && TestModeValues.CONTROL_PANEL_MOTOR_ENABLED == true) {

            } else if (TestModeValues.CONTROL_PANEL_PNEUMATICS_ENABLED == true
                    && TestModeValues.CONTROL_PANEL_MOTOR_ENABLED == true) {

            } else {

            }
        } else {

        }
    }

    private void initializeThrower() {
        if (Constants.IS_THROWER_SUBSYSTEM_IN_USE) {
            if (TestModeValues.THROWER_UPPER_SHAFT_MOTORS == true
                    && TestModeValues.THROWER_LOWER_SHAFT_MOTOR == false) {

            } else if (TestModeValues.THROWER_UPPER_SHAFT_MOTORS == false
                    && TestModeValues.THROWER_LOWER_SHAFT_MOTOR == true) {

            } else if (TestModeValues.THROWER_UPPER_SHAFT_MOTORS == true
                    && TestModeValues.THROWER_LOWER_SHAFT_MOTOR == true) {

            } else {

            }
        } else {

        }
    }

    private void initializeTunnel() {
        if (Constants.IS_TUNNEL_SUBSYSTEM_IN_USE) {

        } else {

        }
    }

    private void initializeClimberCommands() {
        if (Constants.IS_CLIMBING_SUBSYSTEM_IN_USE) {
            if (TestModeValues.RIGHT_CLIMBER) {

            } else if (TestModeValues.LEFT_CLIMBER) {

            }
        } else {

        }
    }

    private void createCommandArrayList() {
        commandObjects.clear();
        try {
            f = c.getDeclaredFields();
            for (int i = 0; i < f.length; i++) {
                Object o = f[i];
                if (o instanceof Command ){
                    commandObjects.add((Command) o);
                }
            }
        } catch (final Throwable e) {
            System.err.println(e);
        }
    }

    private void resetCommands() {
        for(int i = 0; i < commandObjects.size(); i++) {
            commandObjects.remove(i);
        }
    }

    private void initializePossibleCommandGroups() {
        
    }

}   