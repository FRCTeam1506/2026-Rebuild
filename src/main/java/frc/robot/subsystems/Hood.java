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

public class Hood extends SubsystemBase {
  private TalonFX hood = new TalonFX(0);

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  /** Creates a new Hood. */
  public Hood() {
    TalonFXConfiguration towerHopperConfigs = new TalonFXConfiguration();
    towerHopperConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    towerHopperConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    towerHopperConfigs.CurrentLimits.StatorCurrentLimit = 70;//100  80

    towerHopperConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    towerHopperConfigs.CurrentLimits.SupplyCurrentLimit = 30;//60  40

    hood.getConfigurator().apply(towerHopperConfigs);


    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 70;//100  80

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;//60  40
    var slot0Configs = config.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 2; //4.8
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;
    hood.getConfigurator().apply(slot0Configs); 
    hood.getConfigurator().apply(config);

    m_motmag.EnableFOC = true;
  }

  public void moveHood(double Position) {
    hood.setControl(m_motmag.withPosition(Position));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
