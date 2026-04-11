package frc.robot.Commands.Align;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class StationaryAutoAimContinuous extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ProfiledPIDController thetaController;
  private final SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  public static boolean isAligned;
  public static boolean atGoal;
  
  public StationaryAutoAimContinuous(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    thetaController = new ProfiledPIDController(
        AlignConstants.aimControllerP, 
        AlignConstants.aimControllerI, 
        AlignConstants.aimControllerD, 
        new TrapezoidProfile.Constraints(AlignConstants.alignMaxCorrectionSpeed, AlignConstants.alignMaxAcceleration)
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(AlignConstants.alignToleranceRadians, 0.01);
  }

  @Override
  public void initialize() {
    atGoal = false;
    var currentState = drivetrain.getState();
    thetaController.reset(
        currentState.Pose.getRotation().getRadians(), 
        currentState.Speeds.omegaRadiansPerSecond
    );
  }

  @Override
  public void execute() {
    Pose2d robotPose = drivetrain.getState().Pose;

    double xDistToGoal = FieldConstants.goalLocation.getX() - robotPose.getX();
    double yDistToGoal = FieldConstants.goalLocation.getY() - robotPose.getY();
    
    double targetAngle = MathUtil.angleModulus(Math.atan2(yDistToGoal, xDistToGoal) + Math.PI);

    double rotationalVelocity = thetaController.calculate(
        robotPose.getRotation().getRadians(), 
        targetAngle
    );

    atGoal = thetaController.atGoal();
    double angleError = Math.abs(MathUtil.angleModulus(robotPose.getRotation().getRadians() - targetAngle));
    isAligned = angleError <= AlignConstants.alignToleranceRadians;

    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, rotationalVelocity)));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
  }
}
