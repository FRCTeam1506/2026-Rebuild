// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
  private TalonFX hood = new TalonFX(HoodConstants.Hood_ID);

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  /** Creates a new Hood. */
  public Hood() {
    TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();
    //hoodConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfigs.CurrentLimits.StatorCurrentLimit = 70;

    hoodConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfigs.CurrentLimits.SupplyCurrentLimit = 30;

    var slot0Configs = hoodConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 2; //4.8
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;
    hood.getConfigurator().apply(slot0Configs); 
    hood.getConfigurator().apply(hoodConfigs);

    m_motmag.EnableFOC = true;
  }

  public void moveHood(double position) {
    hood.setControl(m_motmag.withPosition(position));
  }

  public void hoodUp() {
    HoodConstants.Tuner_Hood_Pos += 0.1;
  }
  public void hoodDown() {
    HoodConstants.Tuner_Hood_Pos -= 0.1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
