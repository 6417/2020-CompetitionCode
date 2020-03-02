/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

import ch.team6417.utils.LatchedBoolean;
import ch.team6417.utils.LatchedBoolean.EdgeDetection;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Motors;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends CommandBase {

  private DriveSubsystem m_driveSubsystem;
  private VisionSubsystem m_visionSubsystem;
  // codesmell
  private static double angleToRotate = 0;
  private boolean firstTargetFound = false;

  /**
   * Creates a new VisionAlignCommand.
   */
  public VisionAlignCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
  //  super(driveSubsystem.turnController, () -> angleToRotate, () -> 0,
  //      (output) -> driveSubsystem.arcadeDrive(0, output), driveSubsystem);
    m_visionSubsystem = visionSubsystem;
    m_driveSubsystem = driveSubsystem;

    addRequirements(m_visionSubsystem, m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.ahrs.reset();
    System.out.println("Vision allign initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_visionSubsystem.newValueReceived() && m_visionSubsystem.getTarget()) {
      System.out.println("new value true");
      firstTargetFound = true;
      angleToRotate = calculateAngleToRotate();
      Robot.ahrs.reset();
    } else if(firstTargetFound == true) {
      angleToRotate -= Math.toRadians(Robot.ahrs.getYaw()%2 * Math.PI);
    } 
      System.out.println("New value false, target detected: " + m_visionSubsystem.getTarget());
      System.out.println("angle to rotate: " + angleToRotate);
      System.out.println("Navx angle: " + Robot.ahrs.getYaw());


      Motors.drivePIDLeft.setReference(Motors.drive_encoder_back_left.getPosition() + angleToRotate * 10, ControlType.kPosition);
      Motors.drivePIDRight.setReference(Motors.drive_encoder_back_right.getPosition() + angleToRotate * 10, ControlType.kPosition);
 //   super.execute();
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
      if(angleToRotate < 0.03 && angleToRotate > -0.03) {
        m_driveSubsystem.arcadeDrive(0.0, 0.0);
        System.out.println("Command finished");
        return true;
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
