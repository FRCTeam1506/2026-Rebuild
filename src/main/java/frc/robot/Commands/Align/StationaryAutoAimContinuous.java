package frc.robot.Commands.Align;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class StationaryAutoAimContinuous extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  //private final ProfiledPIDController thetaController;
  //private final SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();
 // SwerveRequest.ApplyRobotSpeeds request;
  private final SwerveRequest.RobotCentricFacingAngle alignRequest = new SwerveRequest.RobotCentricFacingAngle();

  public static boolean isAligned;
  public static boolean atGoal;
  
  public StationaryAutoAimContinuous(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    // thetaController = new ProfiledPIDController(
    //     AlignConstants.aimControllerP, 
    //     AlignConstants.aimControllerI, 
    //     AlignConstants.aimControllerD, 
    //     new TrapezoidProfile.Constraints(AlignConstants.alignMaxCorrectionSpeed, AlignConstants.alignMaxAcceleration)
    // );
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // thetaController.setTolerance(AlignConstants.alignToleranceRadians, 0.01);

    alignRequest.HeadingController.setPID(
        AlignConstants.aimControllerP, 
        AlignConstants.aimControllerI, 
        AlignConstants.aimControllerD
    );
    alignRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    alignRequest.HeadingController.setTolerance(AlignConstants.alignToleranceRadians, 0.01);
  }

  @Override
  public void initialize() {
    atGoal = false;
    var currentState = drivetrain.getState();
    // thetaController.reset(
    //     currentState.Pose.getRotation().getRadians(), 
    //     currentState.Speeds.omegaRadiansPerSecond
    // );
  }

  @Override
  public void execute() {
    Pose2d robotPose = drivetrain.getState().Pose;

    double xDistToGoal = FieldConstants.goalLocation.getX() - robotPose.getX();
    double yDistToGoal = FieldConstants.goalLocation.getY() - robotPose.getY();
    
    //double targetAngle = MathUtil.angleModulus(Math.atan2(yDistToGoal, xDistToGoal) + Math.PI);
    Rotation2d targetAngle = new Rotation2d(MathUtil.angleModulus(Math.atan2(yDistToGoal, xDistToGoal)));


    // double rotationalVelocity = thetaController.calculate(
    //     robotPose.getRotation().getRadians(), 
    //     targetAngle
    // );

    //atGoal = thetaController.atGoal();
    var alliance = DriverStation.getAlliance();
    boolean allianceColor = alliance.isPresent() && alliance.get() == Alliance.Red;
      if (allianceColor == false) {
      targetAngle = targetAngle.plus(Rotation2d.fromDegrees(180));
    }
    atGoal = alignRequest.HeadingController.atSetpoint();
    //double angleError = Math.abs(MathUtil.angleModulus(robotPose.getRotation().getRadians() - targetAngle));
    //isAligned = angleError <= AlignConstants.alignToleranceRadians;

    //drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, rotationalVelocity)));
    drivetrain.setControl(alignRequest.withTargetDirection((targetAngle)));

  }

  @Override
  public void end(boolean interrupted) {
    //drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
  }
}
