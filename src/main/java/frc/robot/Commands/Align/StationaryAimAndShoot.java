package frc.robot.Commands.Align;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ejml.equation.Equation;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.EquationConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class StationaryAimAndShoot extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;
  private final Hopper hopper;
    private final ProfiledPIDController thetaController;
      SwerveRequest.ApplyRobotSpeeds request;



  //Pretty cool CTRE thing - https://api.ctr-electronics.com/phoenix6/2024-alpha/java/com/ctre/phoenix6/mechanisms/swerve/SwerveRequest.FieldCentricFacingAngle.html
  private final SwerveRequest.FieldCentricFacingAngle alignRequest = 
      new SwerveRequest.FieldCentricFacingAngle();

  private Translation2d goalLocation;
  private boolean isRed;
  double rotationalVelocity;

  public StationaryAimAndShoot(CommandSwerveDrivetrain drivetrain, Shooter shooter, Hopper hopper) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.hopper = hopper;

    thetaController = new ProfiledPIDController(AlignConstants.aimControllerP, AlignConstants.aimControllerI, AlignConstants.aimControllerD, new TrapezoidProfile.Constraints(AlignConstants.alignMaxCorrectionSpeed, AlignConstants.alignMaxAcceleration));
    thetaController.enableContinuousInput(-Math.PI, Math.PI); 
    addRequirements(drivetrain, shooter, hopper);

    alignRequest.HeadingController.setPID(
        AlignConstants.aimControllerP, 
        AlignConstants.aimControllerI, 
        AlignConstants.aimControllerD
    );
    alignRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    request = new SwerveRequest.ApplyRobotSpeeds();

  }

  @Override
  public void initialize() {
    var allianceColor = DriverStation.getAlliance();
    isRed = allianceColor.isPresent() && allianceColor.get() == Alliance.Red;

    goalLocation = isRed ? 
        new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY) : 
        new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drivetrain.getState().Pose;

    // Shooter Distance and RPS
    double distance = robotPose.getTranslation().getDistance(goalLocation);
    double targetRPS = EquationConstants.calculateRPS(distance);
    //shooter.setShooterRPS(targetRPS);
        // shooter.setShooterRPS(EquationConstants.quadraticRPS(distance)); //For quadratic regression formula (Just in case).

    // Find heading to goal
    double xDistToGoal = goalLocation.getX() - robotPose.getX();
    double yDistToGoal = goalLocation.getY() - robotPose.getY();
    Rotation2d goalHeading = new Rotation2d(Math.atan2(yDistToGoal, xDistToGoal));
    rotationalVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), goalHeading.getRadians() + Math.PI);


    // Set heading
    // drivetrain.setControl(alignRequest
    //     .withVelocityX(0)
    //     .withVelocityY(0)
    //     .withTargetDirection(goalHeading));
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(-0, 0, rotationalVelocity)));


    //goals attained?
    //boolean isAligned = Math.abs(robotPose.getRotation().minus(goalHeading).getDegrees()) <= AlignConstants.alignToleranceDegrees;
    boolean isAligned = Math.abs((robotPose.getRotation().getDegrees() + 180) - goalHeading.getDegrees()) <= AlignConstants.alignToleranceDegrees;
    boolean isSpunUp = shooter.isAtVelocity(targetRPS, ShooterConstants.kRPSTolerance);

    System.out.println("current heading" + robotPose.getRotation());
    System.out.println("goal heading " + goalHeading.getDegrees());
    System.out.println("off by" + Math.abs(robotPose.getRotation().getDegrees() - goalHeading.getDegrees()));

    if (isAligned == true) {
      CommandScheduler.getInstance().schedule(new AutoShoot(shooter, hopper));
      System.out.println("isAligned true");
    }


  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopAllShooters();
  }
}