// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.Intake.IntakeInPower;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX intake = new TalonFX(IntakeConstants.Intake_Motor_ID);
  private TalonFX intakeLift = new TalonFX(IntakeConstants.Intake_Lift_Motor_ID);
  private CANcoder liftEncoder = new CANcoder(IntakeConstants.Intake_Lift_Encoder_ID);
  //private final DigitalInput extendedSwitch = new DigitalInput(IntakeConstants.EXTENDED_SWITCH_DIO);
  //private final DigitalInput retractedSwitch = new DigitalInput(IntakeConstants.RETRACTED_SWITCH_DIO);

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(12);

  boolean out;

  public Intake() {
      // final MotionMagicVoltage m_motmag = new MotionMagicVoltage(12);
      CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
      encoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      liftEncoder.getConfigurator().apply(encoderConfigs);

      TalonFXConfiguration liftConfig = new TalonFXConfiguration();
      liftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      liftConfig.CurrentLimits.StatorCurrentLimit = 95;
      liftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = 105;
      intakeLift.getConfigurator().apply(liftConfig);
  
      intake.getConfigurator().apply(config);

      var motionMagicConfigs = config.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 50;//180 // 80 rps cruise velocity //60 rps gets to L4 in 1.92s //100 //160 //220 before 3/20 bc elevator maltensioned //220 FRCC
      motionMagicConfigs.MotionMagicAcceleration = 60; //260 // 160 rps/s acceleration (0.5 seconds) //220
      motionMagicConfigs.MotionMagicJerk = 3200; // 1600 rps/s^2 jerk (0.1 seconds)

      var slot0Configs = config.Slot0;
      slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
      slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
      // PID runs on position
      slot0Configs.kP = 0.8; //4.8
      slot0Configs.kI = 0;
      slot0Configs.kD = 0.1;

      config.Slot0 = slot0Configs;
      //config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      m_motmag.EnableFOC = true;//from 2025 code


      intakeLift.getConfigurator().apply(motionMagicConfigs);
      intakeLift.getConfigurator().apply(slot0Configs); 
      intake.getConfigurator().apply(slot0Configs); 
      intakeLift.setNeutralMode(NeutralModeValue.Brake);

  }

  public void runIntake(double speed) {
    intake.set(speed);
  }

  public void stopIntake() {
    intake.set(0);
  }

  public void runIntakeLift(double speed) {
    intakeLift.set(speed);
  }

  public void stopIntakeLift() {
    intakeLift.set(0);
  }

  public void setIntakeLift(double Pos) {
    intakeLift.setControl(m_motmag.withPosition(Pos));
  }

  // public boolean isFullyExtended() {
  //   return !extendedSwitch.get();
  // }

  // public boolean isFullyIn() {
  //   return !retractedSwitch.get();
  // }

  public void raiseIntake() {
    setIntakeLift(IntakeConstants.intakeUpPosition);
  }

  public void lowerIntake() {
    setIntakeLift(IntakeConstants.intakeLoweredPosition);
  } 


  @Override
  public void periodic() {
    // Implement when limit switches are put on (if)
    // if (isFullyIn()) {
    //   intakeLift.setPosition(0);
    // }
    // if(isFullyExtended()) {
    //   intakeLift.setPosition(IntakeConstants.intakeLoweredPosition);
    // }

    // SmartDashboard.putBoolean("Intake/Extended Switch", isFullyExtended());
    // SmartDashboard.putBoolean("Intake/Retracted Switch", isFullyIn());
    // This method will be called once per scheduler run
  }
}
