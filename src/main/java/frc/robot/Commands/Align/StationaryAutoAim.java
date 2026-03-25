package frc.robot.Commands.Align;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

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
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
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

  double rotationalVelocity;
  double toRadians;
  public StationaryAutoAim(CommandSwerveDrivetrain drivetrain) {
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
    // Determine alliance once at start to select the correct goal
    thetaController.setTolerance(AlignConstants.alignToleranceRadians);
    var alliance = DriverStation.getAlliance();
    isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
    
    goalLocation = isRed ? 
        new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY) : 
        new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);
        thetaController.reset(0);;
  }

  @Override
  public void execute() {
    System.out.println("stationary auto aim");
    Pose2d robotPose = drivetrain.getState().Pose;

    double xDistToGoal = goalLocation.getX() - robotPose.getX();
    double yDistToGoal = goalLocation.getY() - robotPose.getY();
    Rotation2d goalHeading = new Rotation2d(Math.atan2(yDistToGoal, xDistToGoal));

    rotationalVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), goalHeading.getRadians() + Math.PI);
    //toRadians = Math.toRadians(rotationalVelocity);

    
    // drivetrain.setControl(alignRequest
    //     .withVelocityX(0)
    //     .withVelocityY(0)
    //     .withRotationalRate(rotationalVelocity));

    isAligned = Math.abs(robotPose.getRotation().getDegrees()) - (goalHeading.getDegrees() * -1) <= AlignConstants.alignToleranceRadians;
    System.out.println(robotPose.getRotation().getDegrees());
    System.out.println(goalHeading.getDegrees());


    AlignConstants.isAligned = isAligned;
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, rotationalVelocity)));
    System.out.println("isALigned" + AlignConstants.isAligned);

    // System.out.println("rotational rate" + toRadians);
    // System.out.println("current heading" + robotPose.getRotation().getDegrees());
    // System.out.println("goal heading" + goalHeading.getDegrees());

    double angleError = Math.abs(robotPose.getRotation().minus(goalHeading).getDegrees());
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.driver.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));

  }
  @Override
  public boolean isFinished() {
    return thetaController.atGoal();
    //return isAligned;
  }
}