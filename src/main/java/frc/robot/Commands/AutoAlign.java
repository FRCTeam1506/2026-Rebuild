// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

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
import frc.robot.Constants.alignCalculations;
import frc.robot.Constants.fieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  private final ProfiledPIDController thetaController;

  private final SwerveRequest.FieldCentric alignRequest;

  double currentHeading;
  double rotationalVelocity;
  double toRadians;
  boolean hub;
  public InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();


  public static double theta;
  public static double thetaX, thetaY;
  public static double toDegree;
  public static double preHeading;
  public static double wantedHeading;
  public static double dist;
  public static Translation2d targetVec;
  public static double distToGoal;

  //Robot Poses
  public static Pose2d robotPose;
  public static double robotPoseX;
  public static double robotPoseY;
  public static double omega;
  public static double heading;

  public static Translation2d rotationalVelocityField;

  //Virtual Robot Poses
  public static double vRobotY;
  public static double vRobotX;
  public static double vRotationalRobotY;
  public static double vRotationalRobotX;
  public static double totalFieldVy;
  public static double totalFieldVx;
  public static Translation2d vRobotPose;
  public static boolean red;

  Translation2d goalLocation = new Translation2d(0, 0);


  Optional<Alliance> alliance = DriverStation.getAlliance();

  /** Creates a new AutoAlign. */
  public AutoAlign(CommandSwerveDrivetrain drivetrain, boolean hub) {
    this.drivetrain = drivetrain;
    this.hub = hub;
    alignRequest = new SwerveRequest.FieldCentric();

    thetaController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(2, 2));
    thetaController.enableContinuousInput(-180, 180); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (alliance.get() == Alliance.Red) {
            red = true;
    } else {
            red = false;
  }

    omega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
    robotPose = drivetrain.getState().Pose;
    robotPoseX = drivetrain.getState().Pose.getX();
    robotPoseY = drivetrain.getState().Pose.getY();

    vRotationalRobotY = alignCalculations.omega * alignCalculations.robotPoseX;
    vRotationalRobotX = -alignCalculations.omega * alignCalculations.robotPoseY;
    rotationalVelocityField = new Translation2d(vRotationalRobotX, vRotationalRobotY).rotateBy(drivetrain.getState().Pose.getRotation());

    totalFieldVy = drivetrain.getState().Speeds.vyMetersPerSecond + rotationalVelocityField.getY();
    totalFieldVx = drivetrain.getState().Speeds.vxMetersPerSecond + rotationalVelocityField.getX();
    
    //Times time of flight
    vRobotY = robotPose.getY() + (totalFieldVy * timeOfFlight.get(fieldConstants.distToGoal));
    vRobotX = robotPose.getX() + (totalFieldVx * timeOfFlight.get(fieldConstants.distToGoal));
    vRobotPose = new Translation2d(vRobotX, vRobotY);

      if(hub == true) {
        if(red == true) {
                  if (robotPoseX > fieldConstants.redLine) {
                      goalLocation = new Translation2d(fieldConstants.goalRedX, fieldConstants.goalRedY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
                  if(robotPoseY < 4 && robotPoseX < fieldConstants.redLine) {//left (from red perspective)
                      goalLocation = new Translation2d(fieldConstants.goalLeftRedX, fieldConstants.goalLeftRedY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
                  if(robotPoseY > 4 && robotPoseX < fieldConstants.redLine) { 
                      goalLocation = new Translation2d(fieldConstants.goalRightRedX, fieldConstants.goalRightRedY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
              }
              else { //blue
                  if (robotPoseX < fieldConstants.blueLine) {
                      goalLocation = new Translation2d(fieldConstants.goalBlueX, fieldConstants.goalBlueY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
                  if(robotPoseY < 4 && robotPoseX > fieldConstants.blueLine) { //left (from red perspective)
                      goalLocation = new Translation2d(fieldConstants.goalLeftBlueX, fieldConstants.goalLeftBlueY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
                  if(robotPoseY > 4 && robotPoseX > fieldConstants.blueLine) {
                      goalLocation = new Translation2d(fieldConstants.goalRightBlueX, fieldConstants.goalRightBlueY);
                      thetaX = goalLocation.getX() - vRobotX;
                      thetaY = goalLocation.getY() - vRobotY;
                  }
                  
              }
      }
      else {
        //Goal only
              if (red == true) {
                  goalLocation = new Translation2d(fieldConstants.goalRedX, fieldConstants.goalRedY);
                  thetaX = goalLocation.getX() - vRobotX;
                  thetaY = goalLocation.getY() - vRobotY;
              } else {
                  goalLocation = new Translation2d(fieldConstants.goalBlueX, fieldConstants.goalBlueY);
                  thetaX = goalLocation.getX() - vRobotX;
                  thetaY = goalLocation.getY() - vRobotY;
              }
      }
        heading = drivetrain.getState().Pose.getRotation().getDegrees();
        theta = Math.toDegrees(Math.atan2(thetaY, thetaX)); //degree value of theta
        targetVec = goalLocation.minus(vRobotPose);
        dist = targetVec.getNorm();
        fieldConstants.distToGoal = dist;

        if (alliance.get() == Alliance.Red) {
            red = true;
        } else {
            red = false;
        }
    wantedHeading = theta;
    currentHeading = drivetrain.getState().Pose.getRotation().getDegrees();

    rotationalVelocity = thetaController.calculate(currentHeading, wantedHeading);
    toRadians = Math.toRadians(rotationalVelocity);

    drivetrain.setControl(alignRequest
            .withVelocityX(RobotContainer.driver.getLeftY())//forwards and backwards? YES
            .withVelocityY(RobotContainer.driver.getLeftX()) 
            .withRotationalRate(toRadians));
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


