package frc.robot.Commands.UnusedCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class StationaryAutoAim extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ProfiledPIDController thetaController;
  
  //Pretty cool CTRE thing - https://api.ctr-electronics.com/phoenix6/2024-alpha/java/com/ctre/phoenix6/mechanisms/swerve/SwerveRequest.FieldCentricFacingAngle.html
  // private final SwerveRequest.FieldCentricFacingAngle alignRequest = 
  //     new SwerveRequest.FieldCentricFacingAngle();
  private final SwerveRequest.FieldCentric alignRequest;

  private Translation2d goalLocation;
  private boolean isRed;
  public static boolean isAligned;
  SwerveRequest.ApplyRobotSpeeds request;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();


  double rotationalVelocity;
  
  public StationaryAutoAim(CommandSwerveDrivetrain drivetrain) { //consider making align degrees tolerance a parameter
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    alignRequest = new SwerveRequest.FieldCentric();
    // alignRequest.HeadingController.setPID(AlignConstants.aimControllerP, AlignConstants.aimControllerI, AlignConstants.aimControllerD);
    // alignRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController = new ProfiledPIDController(AlignConstants.aimControllerP, AlignConstants.aimControllerI, AlignConstants.aimControllerD, new TrapezoidProfile.Constraints(AlignConstants.alignMaxCorrectionSpeed, AlignConstants.alignMaxAcceleration));
    thetaController.enableContinuousInput(-Math.PI, Math.PI); 
    request = new SwerveRequest.ApplyRobotSpeeds();
    
  }

  @Override
  public void initialize() {
    var currentRotation = drivetrain.getState().Pose.getRotation().getRadians();
    var currentVelocity = drivetrain.getState().Speeds.omegaRadiansPerSecond;
    thetaController.reset(currentRotation, currentVelocity);

    // Determine alliance once at start to select the correct goal
    thetaController.setTolerance(AlignConstants.alignToleranceRadians, 0.01);
    var alliance = DriverStation.getAlliance();
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    
    goalLocation = isRed ? 
        new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY) : 
        new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drivetrain.getState().Pose;

    double xDistToGoal = goalLocation.getX() - robotPose.getX();
    double yDistToGoal = goalLocation.getY() - robotPose.getY();
    Rotation2d goalHeading = new Rotation2d(Math.atan2(yDistToGoal, xDistToGoal));

    double targetAngle = MathUtil.angleModulus(goalHeading.getRadians() + Math.PI);
    rotationalVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), targetAngle);

    // drivetrain.setControl(alignRequest
    //     .withVelocityX(0)
    //     .withVelocityY(0)
    //     .withRotationalRate(rotationalVelocity));

    //Not being used, old version - isAligned = Math.abs(robotPose.getRotation().getDegrees()) - (goalHeading.getDegrees() * -1) <= AlignConstants.alignToleranceRadians;
    // Correct isAlign code if ever used:
    double angleError = Math.abs(MathUtil.angleModulus(robotPose.getRotation().getRadians() - targetAngle));
    isAligned = angleError <= AlignConstants.alignToleranceRadians;
    //  AlignConstants.isAligned = isAligned;
     
    //AlignConstants.isAligned = isAligned;
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, rotationalVelocity)));

  }

  @Override
  public void end(boolean interrupted) {
    //CommandScheduler.getInstance().;
    // drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    
  }
  @Override
  public boolean isFinished() {
    return thetaController.atGoal();

    //return isAligned;
  }
}