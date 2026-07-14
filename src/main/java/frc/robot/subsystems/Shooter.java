// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EquationConstants;
import frc.robot.FieldConstants;
import frc.robot.Constants.PresetShots;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SimConstants;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

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

  // Sim-only: the intake we pull Fuel from, and a rate limiter so we launch a stream of pieces
  // rather than one per loop. Both are harmless on the real robot.
  private final Intake intake;
  private final Timer shotTimer = new Timer();

  public Shooter(CommandSwerveDrivetrain drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    shotTimer.start();

    TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();

    final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);


    shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfigs.CurrentLimits.StatorCurrentLimit = 70;//100  80

    shooterConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfigs.CurrentLimits.SupplyCurrentLimit = 45;//60  40
    
    var motionMagicConfigs = shooterConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 220; // 80 rps cruise velocity //60 rps gets to L4 in 1.92s //100 //160 //220 before 3/20 bc elevator maltensioned //220 FRCC
    motionMagicConfigs.MotionMagicAcceleration = 260; // 160 rps/s acceleration (0.5 seconds) //220
    motionMagicConfigs.MotionMagicJerk = 3200; // 1600 rps/s^2 jerk (0.1 seconds)

    // set slot 0 gains
    var slot0Configs = shooterConfigs.Slot0;
    
    slot0Configs.kS = 0.26; // Voltage to break static friction (typical range 0.1 - 0.3)
    slot0Configs.kV = 0.11; // Voltage per RPS (approx 11-12V for ~100 RPS / 6000 RPM)
    slot0Configs.kP = 0.5; // Proportional gain (start low; 4 motors have massive torque)
    slot0Configs.kI = 0.0;  // Leave at 0 to avoid oscillation/overshoot
    slot0Configs.kD = 0.01; // Tiny bit of D can help dampen the 4-motor kickback

  
    m_motmag.EnableFOC = true;

    shooterTop.getConfigurator().apply(shooterConfigs);
    shooterRight.getConfigurator().apply(shooterConfigs);
    shooterLeft.getConfigurator().apply(shooterConfigs);
    //shooterRightBack.getConfigurator().apply(shooterConfigs);

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

  public void upPower() {
    PresetShots.tunerPower += 0.25;
  }
  public void downPower() {
    PresetShots.tunerPower -= 0.25;
  }

  public double avgShooterSpeed() { //Three motors
    return (shooterTop.getVelocity().getValueAsDouble() + shooterRight.getVelocity().getValueAsDouble() + shooterLeft.getVelocity().getValueAsDouble()) / 3.0;
  }


  public boolean isAtVelocity(double targetRPS, double tolerance) {
    //original isAtVelocity:
    //double currentRPS = avgShooterSpeed();
    //return Math.abs(currentRPS - targetRPS) <= tolerance;
    // new isatvelocity:
      double currentRPS = avgShooterSpeed();
      boolean isWithinTolerance = Math.abs(currentRPS - targetRPS) <= tolerance;
      return velocityDebouncer.calculate(isWithinTolerance);
      //essentially starts a timer from when when isWithinTolerance is true and only returns true if it continues to be true for 0.1 seconds
     
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("set shooter speed", EquationConstants.calculateRPS(FieldConstants.distToGoal));
    double[] FieldLocation = {FieldConstants.goalLocation.getMeasureX().baseUnitMagnitude(), FieldConstants.goalLocation.getMeasureY().baseUnitMagnitude()};
    SmartDashboard.putNumber("distance to goal", dist);
    SmartDashboard.putNumberArray("goal", FieldLocation);
    SmartDashboard.putNumber("Tuner Power", PresetShots.tunerPower);
    SmartDashboard.putNumber("Shooter Power", avgShooterSpeed());
    // This method will be called once per scheduler run

    simShootPeriodic();
  }

  /**
   * Sim-only: while the shooter is spun up and the intake holds Fuel, launch a Fuel projectile on a
   * rate-limited cadence. maple-sim's RebuiltFuelOnFly figures out whether it lands in the HUB or
   * falls to the floor. Everything here no-ops on a real robot.
   */
  private void simShootPeriodic() {
    if (!Utils.isSimulation()) return;

    IntakeSimulation intakeSim = intake.getIntakeSim();
    Pose2d simPose = drivetrain.getSimulatedPose();
    if (intakeSim == null || simPose == null) return;

    boolean firing = avgShooterSpeed() > SimConstants.kShootRpsThreshold
        && intakeSim.getGamePiecesAmount() > 0
        && shotTimer.hasElapsed(SimConstants.kShotPeriodSec);
    if (!firing) return;

    intakeSim.obtainGamePieceFromIntake(); // consume one held Fuel

    // Field-relative chassis speed so a moving robot imparts momentum to the shot.
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, simPose.getRotation());

    SimulatedArena.getInstance().addGamePieceProjectile(new RebuiltFuelOnFly(
        simPose.getTranslation(),              // robot position on the field
        SimConstants.kShooterOffsetOnRobot,    // shooter offset in the robot frame
        fieldSpeeds,                           // robot velocity (momentum)
        simPose.getRotation(),                 // shooter faces the way the robot faces
        SimConstants.kShooterHeight,           // launch height
        MetersPerSecond.of(avgShooterSpeed() * SimConstants.kLaunchSpeedPerRps), // RPS -> launch speed
        SimConstants.kShooterPitch));          // launch pitch

    shotTimer.reset();
  }
}
