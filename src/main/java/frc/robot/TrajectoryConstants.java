/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * Add your docs here.
 */
public class TrajectoryConstants {

    public static final class DriveConstants {

    public static final double kTrackwidthMeters = 0.50466;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kGearboxRatio = 10.71;
    public static final double kWheelCircumferenceMeters = 0.4787787204060999;
    public static final double kWheelRPMinMeterPerSeconds = 1.0 / kWheelCircumferenceMeters / 60.0 / kGearboxRatio;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
//        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
        1.0/ kGearboxRatio * kWheelCircumferenceMeters;
    public static final boolean kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.17;
    public static final double kvVoltSecondsPerMeter = 2.82;
    public static final double kaVoltSecondsSquaredPerMeter = 0.539;

    // Example value only - as above, this must be tuned for your drive!
    // public static final double kPDriveVel = 2.1;
    public static final double kPDriveVel = 0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.8;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
  
}
