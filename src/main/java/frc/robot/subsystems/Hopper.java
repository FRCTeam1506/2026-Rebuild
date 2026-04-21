// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  //][\private TalonFX extender = new TalonFX(HopperConstants.Extender_Motor_ID);
  private TalonFX hopper = new TalonFX(HopperConstants.Hopper_Motor_ID);
  private TalonFX towerHopper = new TalonFX(HopperConstants.Tower_Hopper_ID);
  private TalonFX towerHopperTwo = new TalonFX(HopperConstants.Tower_Hopper_Two_ID);

  public Hopper() {
    TalonFXConfiguration towerHopperConfigs = new TalonFXConfiguration();
    towerHopperConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    towerHopperConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    towerHopperConfigs.CurrentLimits.StatorCurrentLimit = 70;//100  80

    towerHopperConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    towerHopperConfigs.CurrentLimits.SupplyCurrentLimit = 30;//60  40

    towerHopper.getConfigurator().apply(towerHopperConfigs);
    towerHopperTwo.getConfigurator().apply(towerHopperConfigs);


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
    hopper.getConfigurator().apply(slot0Configs); 
    hopper.getConfigurator().apply(config);
    towerHopper.getConfigurator().apply(config);
    towerHopperTwo.getConfigurator().apply(config);
  }

  public void runHopper(double speed) {
    hopper.set(speed);
    towerHopper.set(speed);
    towerHopperTwo.set(-speed);
  }
  public void stopHopper() {
    hopper.set(0);
    towerHopper.set(0);
    towerHopperTwo.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
