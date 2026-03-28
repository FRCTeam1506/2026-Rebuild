// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EquationConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PresetShots;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FieldConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX shooterLeft = new TalonFX(ShooterConstants.Shooter_Left_ID);
  //private TalonFX shooterRightBack = new TalonFX(ShooterConstants.Right_Back_Shooter_ID);
  private TalonFX shooterTop = new TalonFX(ShooterConstants.Top_Shooter_ID);
  private TalonFX shooterRight = new TalonFX(ShooterConstants.Shooter_Right_ID);

  //More info on debouncer: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/debouncer.html
  private final Debouncer velocityDebouncer = new Debouncer(0.1, DebounceType.kRising);

  final VelocityVoltage speedControl = new VelocityVoltage(0);
  Translation2d targetVec;
  double dist;
  CommandSwerveDrivetrain drivetrain;
  public Shooter(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

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
    
    slot0Configs.kS = 0.25; // Voltage to break static friction (typical range 0.1 - 0.3)
    slot0Configs.kV = 0.11; // Voltage per RPS (approx 11-12V for ~100 RPS / 6000 RPM)
    slot0Configs.kP = 0.5; //.5 // Proportional gain (start low; 4 motors have massive torque)
    slot0Configs.kI = 0.0;  // Leave at 0 to avoid oscillation/overshoot
    slot0Configs.kD = 0.01; // Tiny bit of D can help dampen the 4-motor kickback

  
    m_motmag.EnableFOC = true;

    shooterTop.getConfigurator().apply(shooterConfigs);
    shooterRight.getConfigurator().apply(shooterConfigs);
    shooterLeft.getConfigurator().apply(shooterConfigs);
    //shooterRightBack.getConfigurator().apply(shooterConfigs);

    shooterPower.put(1.0, 1.0);
  }

  public void runAllShootersSpeed(double speed) {
    shooterTop.set(speed);
    shooterRight.set(speed);
    shooterLeft.set(speed);
  }

  public void setShooterRPS(double speed) {
    shooterTop.setControl(speedControl.withVelocity(speed));
    shooterRight.setControl(speedControl.withVelocity(speed));
    shooterLeft.setControl(speedControl.withVelocity(speed));
  }

  public void stopAllShooters() {
    shooterTop.set(0);
    shooterRight.set(0);
    shooterLeft.set(0);
  }

  public void autoShooterPower() {
    shooterTop.set(FieldConstants.distToGoal);
    shooterRight.set(FieldConstants.distToGoal);
    shooterLeft.set(FieldConstants.distToGoal);
    //shooterRightBack.set(FieldConstants.distToGoal);
  }

  public void upPower() {
    PresetShots.tunerPower += 0.25;
  }
  public void downPower() {
    PresetShots.tunerPower -= 0.25;
  }

  public double avgShooterSpeed() { //Three motors
    return (shooterTop.getVelocity().getValueAsDouble() + shooterRight.getVelocity().getValueAsDouble() + shooterLeft.getVelocity().getValueAsDouble()) / 3.0;
  }

  // public double singleMotorSpeed() { //Three motors
  //   return (shooterTop.getVelocity().getValueAsDouble());
  // }

  public boolean isAtVelocity(double targetRPS, double tolerance) {
    double currentRPS = avgShooterSpeed();
    return Math.abs(currentRPS - targetRPS) <= tolerance;
    /* new isatvelocity:
     * double currentRPS = avgShooterSpeed();
    boolean isWithinTolerance = Math.abs(currentRPS - targetRPS) <= tolerance;
    return velocityDebouncer.calculate(isWithinTolerance);
      essentially starts a timer from when when isWithinTolerance is true and only returns true if it continues to be true for 0.1 seconds
     */
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("set shooter speed", EquationConstants.calculateRPS(FieldConstants.distToGoal));
    targetVec = Constants.FieldConstants.goalLocation.minus(drivetrain.getState().Pose.getTranslation());
    dist = targetVec.getNorm();
    FieldConstants.distToGoal = dist;
    double[] FieldLocation = {FieldConstants.goalLocation.getMeasureX().baseUnitMagnitude(), FieldConstants.goalLocation.getMeasureY().baseUnitMagnitude()};
    SmartDashboard.putNumber("distance to goal", dist);
    SmartDashboard.putNumberArray("goal", FieldLocation);


    SmartDashboard.putNumber("Tuner Power", PresetShots.tunerPower);
    SmartDashboard.putNumber("Shooter Power", avgShooterSpeed());
    // This method will be called once per scheduler run
  }
}
