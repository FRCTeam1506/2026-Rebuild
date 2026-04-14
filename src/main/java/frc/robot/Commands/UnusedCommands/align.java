package frc.robot.Commands.UnusedCommands;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class align extends Command {
  CommandSwerveDrivetrain drivetrain;
  /** Creates a new vision2. */
  // SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();
  double x,y,theta;
  boolean finished = false;
  double threshold = 3;  
  SwerveRequest.ApplyRobotSpeeds request;


  double initialX, initialArea, initialYaw, gyroGoal;

  PIDController controller = new PIDController(5, 0, 0);
  // PIDSubsystem

  Translation2d goalLocation;
  Pose2d robotPose;
  //SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();
  double goalHeading;

  public align(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      robotPose = drivetrain.getState().Pose;

    // initialX = Vision.x;
    finished = false;

    initialX = LimelightHelpers.getTX("limelight");
    // initialYaw = TunerConstants.DriveTrain.getPigeon2().getAngle();
    initialYaw = drivetrain.getState().Pose.getRotation().getDegrees();
    gyroGoal = initialYaw - initialX;

    if((Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - gyroGoal) < 2)){
      finished = true;
    }

    // System.out.println(initialX);
    // System.out.println(initialYaw);
    // System.out.println(gyroGoal);
    // System.out.println(gyroGoal % 360);

    
    // Pose2d targetPose2d = new Pose2d(0,0,Rotation2d.fromDegrees(gyroGoal));
    // System.out.println(targetPose2d.getRotation());
    // System.out.println(TunerConstants.DriveTrain.getPigeon2().getAngle() + " gyro angle");
    // System.out.println(Vision.zshot);

    goalLocation = new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);
    double xDistToGoal = goalLocation.getX() - robotPose.getX();
    double yDistToGoal = goalLocation.getY() - robotPose.getY();
    goalHeading = Math.atan2(yDistToGoal, xDistToGoal);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = drivetrain.getState().Pose;

    // request.HeadingController.setPID(0.8, 0.0025, 0.0);
    // request.HeadingController.setPID(5, 0.0, 0);
    // request.Deadband = 3;
    // request.RotationalDeadband = 2;
    // drivetrain.setControl(request.withTargetDirection(Rotation2d.fromDegrees(Math.toDegrees(goalHeading))));
    //double rotationOutput = thetaController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), Vision.angles[tagId]);   

    //drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(-0,0,rotationOutput)));

    // finished = true;
    // TunerConstants.DriveTrain.setControl(request);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds stop = new ChassisSpeeds(0,0, 0);
    //drivetrain.setControl(request.withTargetDirection(0));
    //TunerConstants.DriveTrain.setControl(request.withSpeeds(stop));

    // TunerConstants.DriveTrain.setControl(request.withTargetDirection(Rotation2d.fromDegrees(185)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(LimelightHelpers.getTV("limelight")){
    //   if(Math.abs(LimelightHelpers.getTX("limelight")) < 3){
    //     return true;
    //   }
    // }
    // return finished;//Math.abs(Vision.x) < 3;

    // if(Math.abs(TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees() - gyroGoal) < 2){
    //   return true;
    // }
    // return false;
    return (Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) < 0.2 && (Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - gyroGoal) < 2)) || finished;
  }
}