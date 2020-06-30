/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Motors;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PneumaticsSubsystem.PneumaticState;

public class ControlPanelSubsystem extends SubsystemBase {

  private final DoubleSolenoid liftSolenoid = new DoubleSolenoid(Constants.PCM_CAN_ID,
      Constants.CONTROL_PANEL_LIFT_SOLENOID_EXTEND_ID, Constants.CONTROL_PANEL_LIFT_SOLENOID_RETRACT_ID);

  private final DoubleSolenoid damperSolenoid = new DoubleSolenoid(Constants.PCM_CAN_ID,
      Constants.CONTROL_PANEL_DAMPING_SOLENOID_EXTEND_ID, Constants.CONTROL_PANEL_DAMPING_SOLENOID_RETRACT_ID);

  private final ColorSensorV3 colorSensor = new ColorSensorV3(Constants.CONTROL_PANEL_COLOR_SENSOR_I2C_PORT);
  /**
   * A Rev Color Match object is used to register and detect known colors. This
   * can be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these are
   * here as a basic example.
   */
/*  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.43, 0.39, 0.16);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
*/

  private final Color kBlueTarget = ColorMatch.makeColor(0.13, 0.43, 0.43);
  private final Color kGreenTarget = ColorMatch.makeColor(0.199, 0.562, 0.239);
  private final Color kRedTarget = ColorMatch.makeColor(0.489, 0.376, 0.134);
  private final Color kYellowTarget = ColorMatch.makeColor(0.3366, 0.5446, 0.1176);


  public enum ControlPanelMode {
    POSITION_CONTROL, ROTATION_CONTROL
  }

  public enum ColorDetected {
    RED, GREEN, BLUE, YELLOW, NONE
  }

  private int colorChanges = 25;
  private int startColorID = 0;
  private int currentColorID = 0;
  private int drivenSteps = 0;
  private int steps = 0;
  private double stepTicks = 5000; 
  private double deadband = 1000;
  private boolean rotationControlFinished = false;
  private boolean positionControlFinished = false;
  private ColorDetected[] controlPanelColors = new ColorDetected[]{ColorDetected.RED, ColorDetected.YELLOW, ColorDetected.BLUE, ColorDetected.GREEN};

  private ControlPanelMode mode = ControlPanelMode.ROTATION_CONTROL;

  private String gameData;

  private boolean cancel = false;

  /**
   * Creates a new ControlPlanelSubsystem.
   */
  public ControlPanelSubsystem() {
    addColorsToColorMatcher(kBlueTarget, kGreenTarget, kRedTarget, kYellowTarget);
    m_colorMatcher.setConfidenceThreshold(0.95);
    super.addChild("Lift Solenoid", liftSolenoid);
    super.addChild("Damper Solenoid", damperSolenoid);
    super.addChild("Control Panel Motor", Motors.control_panel_motor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Control Panel Encoder", Motors.control_panel_motor.getSelectedSensorPosition());
    SmartDashboard.putNumber("colorchanges", colorChanges);
    if(RobotContainer.driveJoystick.getRawButton(7)) {
      enableRotationControl();
    } else if(RobotContainer.driveJoystick.getRawButton(8)) {
      enablePositionControl();
    }
    if(RobotContainer.driveJoystick.getPOV() == 90) {  
      Motors.control_panel_motor.set(Constants.CONTROL_PANEL_MANUAL_TURN_SPEED);
    } else if(RobotContainer.driveJoystick.getPOV() == 270) {
      Motors.control_panel_motor.set(-Constants.CONTROL_PANEL_MANUAL_TURN_SPEED);
    } else {
      Motors.control_panel_motor.stopMotor();
    }
  }

  public void addColorsToColorMatcher(Color... colors) {
    for(Color color : colors) {
      m_colorMatcher.addColorMatch(color);
    }
  }

  public ColorDetected getColor() {
    ColorMatchResult match = m_colorMatcher.matchColor(colorSensor.getColor());
    ColorDetected color = ColorDetected.NONE;
    if (match == null) {
      return ColorDetected.NONE;
    } else if (match.color == kBlueTarget) {
      color = ColorDetected.BLUE;
    } else if (match.color == kRedTarget) {
      color = ColorDetected.RED;
    } else if (match.color == kGreenTarget) {
      color = ColorDetected.GREEN;
    } else if (match.color == kYellowTarget) {
      color = ColorDetected.YELLOW;
    }
    return color;
  }

  public String getRGB() {
    Color color = colorSensor.getColor();
    return String.format("%f %f %f", color.red, color.green, color.blue);
  }

  void set(final DoubleSolenoid cylinder, final PneumaticState state) {
    switch (state) {
    case OFF:
      cylinder.set(Value.kOff);
      break;

    case FORWARD:
      cylinder.set(Value.kForward);
      break;

    case REVERSE:
      cylinder.set(Value.kReverse);
      break;
    }
  }

  public void extendLift() {
    set(liftSolenoid, PneumaticState.FORWARD);
  }

  public void retractLift() {
    set(liftSolenoid, PneumaticState.REVERSE);
  }

  public void closeLift() {
    set(liftSolenoid, PneumaticState.OFF);
  }

  public void extendDamper() { 
//    set(damperSolenoid, PneumaticState.FORWARD);
  }

  public void retractDamper() {
//    set(damperSolenoid, PneumaticState.REVERSE);
  }

  public void closeDamper() {
//    set(damperSolenoid, PneumaticState.OFF);
  }

  /* Motor */
  public int getSensorPos() {
    return Motors.control_panel_motor.getSelectedSensorPosition();
  }

  public void setSensorPos(int pos, int timeout) {
    Motors.control_panel_motor.setSelectedSensorPosition(pos, 0, timeout);
  }

  public void resetSensorPos() {
    setSensorPos(0, 30);
  }

  public void setSpeed(double speed) {
    Motors.control_panel_motor.set(ControlMode.PercentOutput, speed);
  }

  public void stopTurn() {
    Motors.control_panel_motor.stopMotor();
  }
  
  public void setMode(ControlPanelMode mode) {
    this.mode = mode;
  }

  public void decideMode() {
    if(mode == ControlPanelMode.ROTATION_CONTROL) {
      setRotationControl();
      System.out.println("Rotation Control");
    } else if(mode == ControlPanelMode.POSITION_CONTROL) {
      setPositionControl();
      System.out.println("Position Control");
    } else {
      System.out.println("Error deciding the Control Panel Mode");
    }
  }

  public void setRotationControl() {
    /* if color sensor is demounted */

    if(getSensorPos() < Constants.TICKS_THREE_ROTATIONS + Constants.ROTATION_CONTROL_SAFETY_TICKS) {
      Motors.control_panel_motor.set(Constants.CONTROL_PANEL_TURN_SPEED);
    } else {
      Motors.control_panel_motor.stopMotor();
      rotationControlFinished = true;
    }

    /* if color sensor is mounted */
    
    // if(colorChanges > 3) {
    //   driveNextSteps();
    // } else {
    //   driveLastSteps();
    // }
  }

  private void driveNextSteps() {
    if(getSensorPos() > steps * 3 * stepTicks - deadband && getSensorPos() < steps * 3 *stepTicks + deadband) {
      System.out.println("in sensor range");
      if(getColor() != ColorDetected.NONE) {
        setCurrentColorID();
        if(currentColorID < startColorID) {
          currentColorID += 4;
        }
        drivenSteps = currentColorID - startColorID;
        colorChanges -= drivenSteps;
        setStartColorID();
        steps++;
     } else {
        Motors.control_panel_motor.stopMotor();
//        colorChanges = 0;
        System.out.println("Error Detecting color");
      }
    } else {
      Motors.control_panel_motor.set(ControlMode.PercentOutput, Constants.CONTROL_PANEL_TURN_SPEED);
    }
  }

  private void driveLastSteps() {
    if(getSensorPos() > (steps - 1) * 3 * stepTicks + colorChanges * stepTicks - deadband) {
      if(getColor() != ColorDetected.NONE) {
        setCurrentColorID();
        if(currentColorID < startColorID) {
          currentColorID += 4;
        }
        drivenSteps = currentColorID - startColorID;
        if(drivenSteps <= colorChanges && drivenSteps > colorChanges - 3) {
          rotationControlFinished = true;
          System.out.println("Rotation Control successfully finished");
        } else {
          rotationControlFinished = false;
          System.out.println("Error reaching the deadband of last color in rotation control");
          Motors.control_panel_motor.stopMotor();
        }
      } else {
        Motors.control_panel_motor.stopMotor();
        colorChanges = 0;
        System.out.println("Error Detecting color");
      }
    } else {
      Motors.control_panel_motor.set(ControlMode.PercentOutput, Constants.CONTROL_PANEL_TURN_SPEED);
    }
  }

  public void setPositionControl() {
    System.out.println("position control");

    /* if color sensor is demounted */
    
    if(RobotContainer.driveJoystick.getRawButton(6)) {
      Motors.control_panel_motor.set(Constants.CONTROL_PANEL_MANUAL_TURN_SPEED);
    } else {
      Motors.control_panel_motor.stopMotor();
    }
    if(RobotContainer.driveJoystick.getRawButton(4)) {
      positionControlFinished = true;
    }



    /* if color sensor is mounted */

    // if(startColorID > readFMS()) {
    //   if(calculateDistantce() - 2 < 0) {
    //     driveStepCount(Constants.CONTROL_PANEL_TURN_SPEED, Math.abs(calculateDistantce() - 2));
    //   } else {
    //     driveStepCount(-Constants.CONTROL_PANEL_TURN_SPEED, Math.abs(calculateDistantce() - 2));
    //   }
    // } else {
    //   if(calculateDistantce() - 2 < 0) {
    //     driveStepCount(-Constants.CONTROL_PANEL_TURN_SPEED, Math.abs(calculateDistantce() - 2));
    //   } else {
    //     driveStepCount(Constants.CONTROL_PANEL_TURN_SPEED, Math.abs(calculateDistantce() - 2));
    //   }
    // }
  }

  private int calculateDistantce() {
    return Math.abs(startColorID - readFMS());
  }

  private void driveStepCount(double speed, int stepCount) {
    if(Math.abs(getSensorPos()) > stepCount * stepTicks - deadband && Math.abs(getSensorPos()) < stepCount * stepTicks + deadband) {
      Motors.control_panel_motor.stopMotor();
      positionControlFinished = true;
      System.out.println("Position Control successfully absolved");
    } else {
      Motors.control_panel_motor.set(ControlMode.PercentOutput, speed);
    }
  }

  public boolean getFinished() {
    if(mode == ControlPanelMode.ROTATION_CONTROL) {
      if(rotationControlFinished == true) {
        return true;
      } else {
        return false;
      }
    } else if(mode == ControlPanelMode.POSITION_CONTROL) {
      if(positionControlFinished == true) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  public boolean setStartColorID() {
    for(int i = 0; i < controlPanelColors.length; i++) {
      if(getColor() == ColorDetected.NONE) {
        return false;
      } else if(controlPanelColors[i] == getColor()) {
        startColorID = i;
        return true;
      }
    }
    return false;
  }

  public void setCurrentColorID() {
    for(int i = 0; i < controlPanelColors.length; i++) {
      if(getColor() == controlPanelColors[i]) {
        currentColorID = i;
      }
    }
  }

  public void resetColorChanges() {
    colorChanges = 25;
  }


  /* FMS Data */
  public int readFMS() {
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0) {
      switch (gameData.charAt(0))
      {
        case 'B' :
          //Blue case code
          return 2;
        case 'G' :
          //Green case code
          return 3;
        case 'R' :
          //Red case code
          return 0;
        case 'Y' :
          //Yellow case code
          return 1;
        default :
          //This is corrupt data
          return 4;
      }
    } else {
      //Code for no data received yet
      return 0;
    }
  }


  /* Getters and Setters */
  public boolean getBottomReed() {
    return Motors.control_panel_motor.isRevLimitSwitchClosed() == 1;
  }

  /* TODO uncomment if front reed is mounted */
  // public boolean getFrontReed() {
  //   return Motors.control_panel_motor.isFwdLimitSwitchClosed() == 1;
  // }

  public double influenceDrive() {
    if(getBottomReed()) {
      return 1.0;
    // } else if (!getFrontReed()) { //TODO remove else if, if upper cilinder is removed
    //   return 0.0;
    }
    return 0.35;
  }

  public void setCancel(boolean cancel) {
    this.cancel = cancel;
  }

  public boolean getCancel() {
    return cancel;
  }

  public void setControlPanelMode(ControlPanelMode controlPanelMode) {
    mode = controlPanelMode;
  }

  public void enableRotationControl() {
    rotationControlFinished = false;
    colorChanges = 25;
    steps = 0;
    drivenSteps = 0;
    setControlPanelMode(ControlPanelMode.ROTATION_CONTROL);
  }

  public void enablePositionControl() {
    positionControlFinished = false;
    setControlPanelMode(ControlPanelMode.POSITION_CONTROL);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Color", () -> getColor().toString(), null);
    builder.addStringProperty("RGB", () -> getRGB(), null);
    builder.addDoubleProperty("Motor", () -> Motors.control_panel_motor.get(), speed -> Motors.control_panel_motor.set(speed));
  }

}
