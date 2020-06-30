/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.team6417.motorcontroller;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Add your docs here.
 */
public class FridoCANSparkMax extends CANSparkMax implements Sendable {
    CANEncoder m_encoder;

    public FridoCANSparkMax(int deviceID, MotorType type) {
        super(deviceID, type);
        m_encoder = super.getEncoder();
    }

    @Override
    public CANEncoder getEncoder() {
        return m_encoder;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CANSparkMax");
        builder.setActuator(true);
        builder.setSafeState(() -> this.stopMotor());
        builder.addDoubleProperty("Encoder Position", () -> this.m_encoder.getPosition(), pos -> this.m_encoder.setPosition(pos));
        builder.addDoubleProperty("Velocity", () -> this.m_encoder.getVelocity(), null);
        builder.addDoubleProperty("Temperature C", () -> this.getMotorTemperature(), null);
    }
    
}
