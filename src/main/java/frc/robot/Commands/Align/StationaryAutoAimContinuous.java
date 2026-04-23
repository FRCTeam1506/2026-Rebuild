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
  private final SwerveRequest.FieldCentricFacingAngle alignRequest = new SwerveRequest.FieldCentricFacingAngle();

  public static boolean isAligned;
  public static boolean atGoal;
  
  public StationaryAutoAimContinuous(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

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
    isAligned = false;
    atGoal = false;
    var currentState = drivetrain.getState();
    alignRequest.HeadingController.reset();
  }

  @Override
  public void execute() {    Pose2d robotPose = drivetrain.getState().Pose;

    double xDist = FieldConstants.goalLocation.getX() - robotPose.getX();
    double yDist = FieldConstants.goalLocation.getY() - robotPose.getY();
    // If the robot points the intake at the goal on Red side but shooter at goal on Blue (or vice versa), 
    /*
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        targetAngleRad += Math.PI; 
    }
    */
    double targetAngle = MathUtil.angleModulus(Math.atan2(yDist, xDist) + Math.PI);
    
    drivetrain.setControl(alignRequest
        .withVelocityX(0)
        .withVelocityY(0)
        .withTargetDirection(new Rotation2d(targetAngle))
    );

    isAligned = alignRequest.HeadingController.atSetpoint();

  }

  @Override
  public void end(boolean interrupted) {
    //drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
  }
}
