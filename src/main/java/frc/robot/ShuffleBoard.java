/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Add your docs here.
 */
public class ShuffleBoard {

    public ShuffleBoard() {}

    private static ShuffleboardTab tab = Shuffleboard.getTab("2020");
    public static NetworkTableEntry throwerUpperMotor =
        tab.add("Thrower Upper Motor speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();

    public static NetworkTableEntry throwerLowerMotor =
        tab.add("Thrower Lower Motor speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();

    public static NetworkTableEntry tunnelMotor =
        tab.add("Tunnel Motor speed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();

    public static NetworkTableEntry shooterVelocity =
        tab.add("Shooter Velocity", 0)
        .withWidget(BuiltInWidgets.kGraph)
        .getEntry();

    public static NetworkTableEntry gripperMotor =
        tab.add("Gripper Velocity", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();

    public static NetworkTableEntry joystick =
        tab.add("Steering Wheel Joystick", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();

        public static NetworkTableEntry joystickMaxSpeed =
        tab.add("Joystick Max Speed", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();

}
