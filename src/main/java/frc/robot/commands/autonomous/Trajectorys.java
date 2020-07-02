/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Commands;
import frc.robot.Robot;
import frc.robot.TrajectoryConstants.AutoConstants;
import frc.robot.TrajectoryConstants.DriveConstants;

/**
 * Add your docs here.
 */
public class Trajectorys {

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand(int command_ID) {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory driveFromLine = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            // new Translation2d(1, 1),
            // new Translation2d(2, 1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
      //  new Pose2d(3, 0, new Rotation2d(0)),
        new Pose2d(1, 0, new Rotation2d(0)),
        // Pass config
        config
    ); 

    Trajectory sixBallAutonomousGrab = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the hole to shot in
        new Pose2d(0, 0, new Rotation2d(0.0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(2.3 , 1.6)
        ),
        // End in the trenchrun after grabing the third ball
        new Pose2d(4.5, 1.7, new Rotation2d(0.0)),
        // Pass config
        config
    );

    Trajectory sixBallAutonomousShoot = TrajectoryGenerator.generateTrajectory(
        //start with a resettet pos. this should be changad to the according position after the first spline to avoid inaccuracy
        new Pose2d(0, 0, new Rotation2d(0.0)),
        List.of(
            //there are no interior waypoints
        ),
        //Drives to the front left corner of the Trenchrun heading to the hole to shot in
        new Pose2d(2.34, 1.0, new Rotation2d(0.1745)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand_default = new RamseteCommand(
        driveFromLine,
        Commands.driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                DriveConstants.kvVoltSecondsPerMeter,
                                DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        Commands.driveSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        Commands.driveSubsystem::tankDriveVolts
        );

        RamseteCommand ramseteCommand_grab = new RamseteCommand(
            sixBallAutonomousGrab,
            Commands.driveSubsystem::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            Commands.driveSubsystem::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            Commands.driveSubsystem::tankDriveVolts
            // Commands.driveSubsystem
        );

        RamseteCommand ramseteCommand_shoot = new RamseteCommand(
        sixBallAutonomousShoot,
        Commands.driveSubsystem::getReversePose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        Commands.driveSubsystem::getReverseWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        Commands.driveSubsystem::reverseTankDriveVolts
        // Commands.driveSubsystem
    );

    if(command_ID == 1) {
        // Run path following command, then stop at the end.
        return ramseteCommand_grab.andThen(() -> Commands.driveSubsystem.tankDriveVolts(0, 0));
    } else if(command_ID == 2) {
        // Run path following command, then stop at the end.
        return ramseteCommand_shoot.andThen(() -> Commands.driveSubsystem.tankDriveVolts(0, 0));
    } else {
        // Run path following command, then stop at the end.
        return ramseteCommand_default.andThen(() -> Commands.driveSubsystem.tankDriveVolts(0, 0));
    }
  }

}
