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
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Motors;
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
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.43, 0.39, 0.16);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public enum ControlPanelMode {
    POSITION_CONTROL, ROTATION_CONTROL
  }

  public enum ColorDetected {
    RED, GREEN, BLUE, YELLOW, NONE
  }


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
    // This method will be called once per scheduler run
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
    set(damperSolenoid, PneumaticState.FORWARD);
  }

  public void retractDamper() {
    set(damperSolenoid, PneumaticState.REVERSE);
  }

  public void closeDamper() {
    set(damperSolenoid, PneumaticState.OFF);
  }

  /* Motor */
  public int getSensorPos() {
    return Motors.control_panel_motor.getSelectedSensorPosition();
  }

  public void setSensorPos(int pos, int timeout) {
    Motors.control_panel_motor.setSelectedSensorPosition(pos, 0, timeout);
  }

  public void resetSensorPos() {
    setSensorPos(0, 30);;
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
    } else if(mode == ControlPanelMode.POSITION_CONTROL) {
      setPositionControl();
    } else {
      System.out.println("Error deciding the Control Panel Mode");
    }
  }

  public void setRotationControl() {
    System.out.println("Rotation control");
    setSpeed(0.2);
  }

  public void setPositionControl() {
    System.out.println("position control");
  }

  public boolean getFinished() {
    if(mode == ControlPanelMode.ROTATION_CONTROL) {
      if(getSensorPos() >= Constants.CONTROL_PANEL_MIN_ROTATIONS_IN_TICKS) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }


  /* FMS Data */
  public int readFMS() {
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0) {
      switch (gameData.charAt(0))
      {
        case 'B' :
          //Blue case code
          return 1;
        case 'G' :
          //Green case code
          return 2;
        case 'R' :
          //Red case code
          return 3;
        case 'Y' :
          //Yellow case code
          return 4;
        default :
          //This is corrupt data
          return 5;
      }
    } else {
      //Code for no data received yet
      return 0;
    }
  }


  /* Getters and Setters */
  public boolean getBottomReed() {
    return Motors.control_panel_motor.isFwdLimitSwitchClosed() == 0;
  }

  public boolean getFrontReed() {
    return Motors.control_panel_motor.isRevLimitSwitchClosed() == 0;
  }

  public void setCancel(boolean cancel) {
    this.cancel = cancel;
  }

  public boolean getCancel() {
    return cancel;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Color", () -> getColor().toString(), null);
    builder.addStringProperty("RGB", () -> getRGB(), null);
    builder.addDoubleProperty("Motor", () -> Motors.control_panel_motor.get(), speed -> Motors.control_panel_motor.set(speed));
  }

}
