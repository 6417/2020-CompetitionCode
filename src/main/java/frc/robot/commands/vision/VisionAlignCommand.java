/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends CommandBase {

  private DriveSubsystem m_driveSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private static double angleToRotate = 0;
  private static double angleToRotateUpdated = 0;
  private boolean firstTargetFound = false;
  private double yaw = 0;
  private double preAngle = 0;
  private boolean didtwice = false;

  /**
   * Creates a new VisionAlignCommand.
   */
  public VisionAlignCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_driveSubsystem = driveSubsystem;

    addRequirements(m_visionSubsystem, m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.ahrs.reset();
    angleToRotate = m_visionSubsystem.getCalculatedAngle();
    firstTargetFound = false;
    didtwice = false;
    preAngle = (-Robot.ahrs.getYaw());
 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_visionSubsystem.newValueReceived() && m_visionSubsystem.getTarget()) {
      System.out.println("new value true");
      firstTargetFound = true;
      yaw = Math.toRadians(-Robot.ahrs.getYaw());
//      preAngle = (Robot.ahrs.getYaw());
    } else if(firstTargetFound == true) {
      angleToRotateUpdated = angleToRotate;
    }
    angleToRotateUpdated = Math.toDegrees(angleToRotate) - (preAngle - (-Robot.ahrs.getYaw()));

      System.out.println("New value false, target detected: " + m_visionSubsystem.getTarget());


      SmartDashboard.putNumber("angle to rotate", Math.toDegrees(angleToRotate));
      SmartDashboard.putNumber("angle to rotate updated: ", (angleToRotateUpdated));
      SmartDashboard.putNumber("NavxAngle", Math.toDegrees(yaw));
      yaw = Math.toRadians(Robot.ahrs.getYaw());
      SmartDashboard.putNumber("Angle", Robot.ahrs.getYaw());
      m_driveSubsystem.arcadeDrive(0,angleToRotateUpdated);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.arcadeDrive(0,0);
    firstTargetFound = false;
    m_visionSubsystem.setVisionAligned(true);
//    m_visionSubsystem.toggleVisionLight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (firstTargetFound) {
      // TODO add return true when in tolerance
      // if(angleToRotateUpdated < 1 && angleToRotateUpdated > -1) {
      //   if(didtwice == false){
      //   m_driveSubsystem.arcadeDrive(0.0, 0.0);
      //   System.out.println("Command finished");
      //   return true;
      //   } else {
      //     m_driveSubsystem.arcadeDrive(0.0, 0.0);
      //     Robot.ahrs.reset();
      //     angleToRotate = m_visionSubsystem.getCalculatedAngle();
      //     firstTargetFound = false;
      //     didtwice = true;
      //     preAngle = (Robot.ahrs.getYaw());
      //     return false;
      //   }
      // } else {
      //   return false;
      // }
    // } else {
    //   return false;
    // }
    return true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("angle to rotate", () -> Math.toDegrees(angleToRotate),
        (angle) -> angleToRotate = Math.toRadians(angle));
  }
}
