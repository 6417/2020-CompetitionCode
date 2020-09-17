/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands;
import frc.robot.Motors;
import frc.robot.ShuffleBoard;
import frc.robot.TrajectoryConstants.AutoConstants;
import frc.robot.TrajectoryConstants.DriveConstants;


public class VisionSubsystem extends SubsystemBase {

  private RamseteCommand ramseteCommand;
  private TrajectoryConfig config;
  private Trajectory generatedTrajectory;

  private DigitalOutput visionLight = new DigitalOutput(1);

  private int X_RES = 640;
  private int Y_RES = 480;
  private double HORIZONTAL_FOV = 62.2;
  private double VERTICAL_FOV = 48.8;

  private double xAngle = 0;
  private double yAngle = 0;

  private NetworkTableEntry XOffset;
  private NetworkTableEntry YOffset;

  private double xOffset;
  private double yOffset;

  private boolean visionAligned = false;

  /**
   * Creates a new VisionSubsystem.
   */
  public VisionSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable table = inst.getTable("vision");

    XOffset = table.getEntry("TargetCenterX");
    YOffset = table.getEntry("TargetCenterY");

    generateRamseteCommand(generateTrajectory());
  }

  @Override
  public void periodic() {
    readData();
    calculateAngles();
    printValues();
    checkAlignement();
//    resetTable(); //in case of non updating values but it needs to be timed otherwise the coprocessor is too slow and the values are just one iteration correct and then reset
    if(Motors.thrower_motor_upper_shaft_right.getEncoder().getVelocity() != 0) {
      visionAligned = false;
    }
  }

  public void printValues() {
    ShuffleBoard.visionLight.setBoolean(visionLight.get());
    SmartDashboard.putNumber("XOffset", xOffset);
    SmartDashboard.putNumber("YOffset", yOffset);
    SmartDashboard.putNumber("XAngle", xAngle);
    SmartDashboard.putNumber("YAngle", yAngle);
  }

  public void readData() {
    xOffset = XOffset.getDouble(X_RES / 2);
    yOffset = YOffset.getDouble(Y_RES / 2);
  }

  public void calculateAngles() {
    xAngle = HORIZONTAL_FOV / X_RES * (xOffset - X_RES / 2);
    yAngle = VERTICAL_FOV / Y_RES * (yOffset - Y_RES / 2);
  }

  private void resetTable() {
    XOffset.setDouble(X_RES / 2);
    YOffset.setDouble(Y_RES / 2);
  }

  private void checkAlignement() {
    if(xAngle < 3 && xAngle > -3) {
      visionAligned = true;
    } else {
      visionAligned = false;
    }
  }

  public double getXOffset() {
    return xOffset;
  }

  public double getYOffset() {
    return yOffset;
  }

  public double getXAngle() {
    return xAngle;
  }

  public double getYAngle() {
    return yAngle;
  }

  public boolean isAligned() {
    return visionAligned;
  }

  public void toggleVisionLight() {
    System.out.println("visionLightActivated");
    visionLight.set(!visionLight.get());
  }


  

  public Trajectory generateTrajectory() {
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
                                   DriveConstants.kDriveKinematics,
                                   10);

    config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);


    generatedTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(

      ),
      new Pose2d(0.2, 0, new Rotation2d(Math.toRadians(-getXAngle()))),
      config
    );

    return generatedTrajectory;

  }


  public RamseteCommand generateRamseteCommand(Trajectory trajectory) {
      ramseteCommand = new RamseteCommand(
      trajectory,
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
    return ramseteCommand;

  }

  public void setRamseteCommand(RamseteCommand input) {
    ramseteCommand = input;
  }

  public RamseteCommand getRamseteCommand() {
    System.out.println(generatedTrajectory.getStates());

    return ramseteCommand;
  }

}
