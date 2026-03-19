// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FieldConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX shooterLeftBack = new TalonFX(ShooterConstants.Left_Back_Shooter_ID);
  private TalonFX shooterRightBack = new TalonFX(ShooterConstants.Right_Back_Shooter_ID);
  private TalonFX shooterLeftFront = new TalonFX(ShooterConstants.Left_Front_Shooter_ID);
  private TalonFX shooterRightFront = new TalonFX(ShooterConstants.Right_Front_Shooter_ID);

  public InterpolatingDoubleTreeMap shooterPower = new InterpolatingDoubleTreeMap();

  final VelocityVoltage speedControl = new VelocityVoltage(0);


  public Shooter() {
    TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
    final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);


    shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfigs.CurrentLimits.StatorCurrentLimit = 100;

    var motionMagicConfigs = shooterConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 220; // 80 rps cruise velocity //60 rps gets to L4 in 1.92s //100 //160 //220 before 3/20 bc elevator maltensioned //220 FRCC
    motionMagicConfigs.MotionMagicAcceleration = 260; // 160 rps/s acceleration (0.5 seconds) //220
    motionMagicConfigs.MotionMagicJerk = 3200; // 1600 rps/s^2 jerk (0.1 seconds)

    // set slot 0 gains
    var slot0Configs = shooterConfigs.Slot0;
    
    // Values based on a typical 1:1 or 2:1 geared 4-motor drum shooter
    slot0Configs.kS = 0.25; // Voltage to break static friction (typical range 0.1 - 0.3)
    slot0Configs.kV = 0.11; // Voltage per RPS (approx 11-12V for ~100 RPS / 6000 RPM)
    slot0Configs.kP = 0.50; // Proportional gain (start low; 4 motors have massive torque)
    slot0Configs.kI = 0.0;  // Leave at 0 to avoid oscillation/overshoot
    slot0Configs.kD = 0.01; // Tiny bit of D can help dampen the 4-motor kickback

  
    m_motmag.EnableFOC = true;

    shooterLeftFront.getConfigurator().apply(shooterConfigs);
    shooterRightFront.getConfigurator().apply(shooterConfigs);
    shooterLeftBack.getConfigurator().apply(shooterConfigs);
    shooterRightBack.getConfigurator().apply(shooterConfigs);



    shooterPower.put(1.0, 1.0);
  }

  public void runAllShootersSpeed(double speed) {
    shooterLeftFront.set(speed);
    shooterRightFront.set(speed);
    shooterLeftBack.set(speed);
    shooterRightBack.set(speed);
  }

  public void setShooterRPS(double speed) {
    shooterLeftFront.setControl(speedControl.withVelocity(speed));
    shooterRightFront.setControl(speedControl.withVelocity(speed));
    shooterLeftBack.setControl(speedControl.withVelocity(speed));
    shooterRightBack.setControl(speedControl.withVelocity(speed));
  }

  public void stopAllShooters() {
    shooterLeftFront.set(0);
    shooterRightFront.set(0);
    shooterLeftBack.set(0);
    shooterRightBack.set(0);
  }

  public void autoShooterPower() {
    shooterLeftFront.set(FieldConstants.distToGoal);
    shooterRightFront.set(FieldConstants.distToGoal);
    shooterLeftBack.set(FieldConstants.distToGoal);
    shooterRightBack.set(FieldConstants.distToGoal);
  }

  public double avgShooterSpeed() {
    return (shooterLeftFront.getVelocity().getValueAsDouble() + shooterRightFront.getVelocity().getValueAsDouble() + shooterLeftBack.getVelocity().getValueAsDouble() + shooterRightBack.getVelocity().getValueAsDouble()) / 4.0;
  }

  public boolean isAtVelocity(double targetRPS, double tolerance) {
    double currentRPS = avgShooterSpeed();
    return Math.abs(currentRPS - targetRPS) <= tolerance;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
