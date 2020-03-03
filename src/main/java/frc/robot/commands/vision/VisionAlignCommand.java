/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends PIDCommand {

  private DriveSubsystem m_driveSubsystem;
  private VisionSubsystem m_visionSubsystem;
  // codesmell
  private static double angleToRotate = 0;
  private static double angleToRotateUpdated = 0;
  private boolean firstTargetFound = false;
  private double yaw = 0;
  private double preAngle = 0;
  private static double offset= 0;
  private boolean didtwice = false;

  /**
   * Creates a new VisionAlignCommand.
   */
  public VisionAlignCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    super(driveSubsystem.turnController, () -> offset, () -> 0.0,
        (output) -> driveSubsystem.arcadeDrive(0, output), driveSubsystem);
    m_visionSubsystem = visionSubsystem;
    m_driveSubsystem = driveSubsystem;

    addRequirements(m_visionSubsystem, m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Vision allign initialized");
    Robot.ahrs.reset();
    angleToRotate = calculateAngleToRotate();
    firstTargetFound = false;
    didtwice = false;
    preAngle = (Robot.ahrs.getYaw());
 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_visionSubsystem.newValueReceived() && m_visionSubsystem.getTarget()) {
      System.out.println("new value true");
      firstTargetFound = true;
//      angleToRotate = calculateAngleToRotate();
      yaw = Math.toRadians(Robot.ahrs.getYaw());
//      preAngle = (Robot.ahrs.getYaw());
      offset = m_visionSubsystem.getOffset();
    } else if(firstTargetFound == true) {
      angleToRotateUpdated = angleToRotate;
    }
    angleToRotateUpdated = Math.toDegrees(angleToRotate) - (preAngle - (Robot.ahrs.getYaw()));

      System.out.println("New value false, target detected: " + m_visionSubsystem.getTarget());


      // Motors.drivePIDLeft.setReference(Motors.drive_encoder_back_left.getPosition() + angleToRotate * 10, ControlType.kPosition);
      // Motors.drivePIDRight.setReference(Motors.drive_encoder_back_right.getPosition() + angleToRotate * 10, ControlType.kPosition);
      SmartDashboard.putNumber("angle to rotate", Math.toDegrees(angleToRotate));
      SmartDashboard.putNumber("offset", offset);
      SmartDashboard.putNumber("angle to rotate updated: ", (angleToRotateUpdated));
      SmartDashboard.putNumber("encoder left", Motors.talon_drive_motor_back_left.getSelectedSensorPosition());
      SmartDashboard.putNumber("encoder right", Motors.talon_drive_motor_front_left.getSelectedSensorPosition());
      SmartDashboard.putNumber("reference left", (double)Motors.talon_drive_motor_back_left.getSelectedSensorPosition() + angleToRotate * 10);
      SmartDashboard.putNumber("reference right", (double)Motors.talon_drive_motor_front_right.getSelectedSensorPosition() - angleToRotate * 10);
      SmartDashboard.putNumber("NavxAngle", Math.toDegrees(yaw));
      yaw = Math.toRadians(Robot.ahrs.getYaw());
      m_driveSubsystem.arcadeDrive(0,angleToRotateUpdated);
//      super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.arcadeDrive(0,0);
    firstTargetFound = false;
    m_visionSubsystem.toggleVisionLight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (firstTargetFound) {
      // TODO add return true when in tolerance
      System.out.println("Offset: " + m_visionSubsystem.getOffset());
      if(angleToRotateUpdated < 1 && angleToRotateUpdated > -1) {
        if(didtwice == false){
        m_driveSubsystem.arcadeDrive(0.0, 0.0);
        System.out.println("Command finished");
        return true;
      } else {
        m_driveSubsystem.arcadeDrive(0.0, 0.0);
        Robot.ahrs.reset();
        angleToRotate = calculateAngleToRotate();
        firstTargetFound = false;
        didtwice = true;
        preAngle = (Robot.ahrs.getYaw());
        return false;
      }
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  private double calculateAngleToRotate() {
    return Math.atan(m_visionSubsystem.getOffset() / m_visionSubsystem.getDistance());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("angle to rotate", () -> Math.toDegrees(angleToRotate),
        (angle) -> angleToRotate = Math.toRadians(angle));
  }
}
