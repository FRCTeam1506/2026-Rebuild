// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Align;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.EquationConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * AutoAim command aligns the robot's drivetrain to a target on the field.
 * @param drivetrain The swerve drivetrain subsystem to control.
 * @param hub Whether to aim at a specific hub configuration (true) or just the main goal (false).
 * It calculates the necessary rotation to face a specific goal or hub 
 * based on the robot's current position and velocity (correcting for time of flight).
 */
public class MovingAutoAimAndShoot extends Command {
  Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;

  private final ProfiledPIDController thetaController;

  private final SwerveRequest.FieldCentric alignRequest;

  double currentHeading;
  double rotationalVelocity;
  double toRadians;
  boolean hub;

  private double theta;
  private double thetaX, thetaY;
  private double toDegree;
  private double preHeading;
  private double wantedHeading;
  private double dist;
  private Translation2d targetVec;
  private double distToGoal;

  //Robot Poses
  public static Pose2d robotPose;
  public static double robotPoseX;
  public static double robotPoseY;
  private static double omega;
  private static double heading;
  private static Translation2d rotationalVelocityField;

  //Virtual Robot Poses
  public static double vRobotY;
  public static double vRobotX;
  private static double vRotationalRobotY;
  private static double vRotationalRobotX;
  private static double totalFieldVy;
  private static double totalFieldVx;
  private static Translation2d vRobotPose;
  private static boolean isRed;

  Translation2d goalLocation = new Translation2d(0, 0);


  Optional<Alliance> alliance = DriverStation.getAlliance();

  double targetRPS;

  /**
   * Creates a new AutoAim command.
   *
   * @param drivetrain The swerve drivetrain subsystem to control.
   * @param hub        Whether to aim at a specific hub configuration (true) or just the main goal (false).
   */
  public MovingAutoAimAndShoot(CommandSwerveDrivetrain drivetrain, Shooter shooter, boolean hub) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.hub = hub;
    alignRequest = new SwerveRequest.FieldCentric();

    thetaController = new ProfiledPIDController(AlignConstants.aimControllerP, AlignConstants.aimControllerI, AlignConstants.aimControllerD, new TrapezoidProfile.Constraints(AlignConstants.alignMaxCorrectionSpeed, AlignConstants.alignMaxAcceleration));
    thetaController.enableContinuousInput(-180, 180); 

    addRequirements(drivetrain, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var allianceColor = DriverStation.getAlliance();
    isRed = allianceColor.isPresent() && allianceColor.get() == Alliance.Red;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    omega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
    robotPose = drivetrain.getState().Pose;
    robotPoseX = drivetrain.getState().Pose.getX();
    robotPoseY = drivetrain.getState().Pose.getY();

    vRotationalRobotY = omega * robotPoseX;
    vRotationalRobotX = -omega * robotPoseY;
    rotationalVelocityField = new Translation2d(vRotationalRobotX, vRotationalRobotY).rotateBy(drivetrain.getState().Pose.getRotation());

    totalFieldVy = drivetrain.getState().Speeds.vyMetersPerSecond + rotationalVelocityField.getY();
    totalFieldVx = drivetrain.getState().Speeds.vxMetersPerSecond + rotationalVelocityField.getX();
    
    //Times time of flight
    vRobotY = robotPose.getY() + (totalFieldVy * EquationConstants.calculateTimeOfFlight(FieldConstants.distToGoal));
    vRobotX = robotPose.getX() + (totalFieldVx * EquationConstants.calculateTimeOfFlight(FieldConstants.distToGoal));
    vRobotPose = new Translation2d(vRobotX, vRobotY);

      if(hub == true) {
        if(isRed == true) {
                  if (robotPoseX > FieldConstants.redLine) {
                      goalLocation = new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
                  if(robotPoseY < 4 && robotPoseX < FieldConstants.redLine) {//left (from red perspective)
                      goalLocation = new Translation2d(FieldConstants.goalLeftRedX, FieldConstants.goalLeftRedY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
                  if(robotPoseY > 4 && robotPoseX < FieldConstants.redLine) { 
                      goalLocation = new Translation2d(FieldConstants.goalRightRedX, FieldConstants.goalRightRedY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
              }
              else { //blue
                  if (robotPoseX < FieldConstants.blueLine) {
                      goalLocation = new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
                  if(robotPoseY < 4 && robotPoseX > FieldConstants.blueLine) { //left (from red perspective)
                      goalLocation = new Translation2d(FieldConstants.goalLeftBlueX, FieldConstants.goalLeftBlueY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
                  if(robotPoseY > 4 && robotPoseX > FieldConstants.blueLine) {
                      goalLocation = new Translation2d(FieldConstants.goalRightBlueX, FieldConstants.goalRightBlueY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
                  
              }
      }
      else {
        //Goal only
              if (isRed == true) {
                  goalLocation = new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY);
                  thetaX = goalLocation.getX() - vRobotX;
                  thetaY = goalLocation.getY() - vRobotY;
              } else {
                  goalLocation = new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);
                  thetaX = goalLocation.getX() - vRobotX;
                  thetaY = goalLocation.getY() - vRobotY;
              }
      }
        //heading = drivetrain.getState().Pose.getRotation().getDegrees(); //Don't need?
        theta = Math.toDegrees(Math.atan2(thetaY, thetaX)); //degree value of theta
        targetVec = goalLocation.minus(vRobotPose);
        // dist = targetVec.getNorm();
        dist = robotPose.getTranslation().getDistance(goalLocation); //New way to get distance
        FieldConstants.distToGoal = dist;

        if (alliance.get() == Alliance.Red) {
            isRed = true;
        } else {
            isRed = false;
        }
    wantedHeading = theta;
    currentHeading = drivetrain.getState().Pose.getRotation().getDegrees();

    rotationalVelocity = thetaController.calculate(currentHeading, wantedHeading);
    toRadians = Math.toRadians(rotationalVelocity);

    drivetrain.setControl(alignRequest
            .withVelocityX(-RobotContainer.driver.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))//forwards and backwards? YES
            .withVelocityY(-RobotContainer.driver.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
            .withRotationalRate(toRadians));

    

    //shooter.setShooterRPS(EquationConstants.calculateRPS(dist)); //Math
    shooter.setShooterRPS(EquationConstants.quadraticRPS(dist)); //Quadratic regression
  }  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(alignRequest
      .withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


