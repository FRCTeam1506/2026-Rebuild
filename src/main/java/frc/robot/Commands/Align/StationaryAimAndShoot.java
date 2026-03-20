package frc.robot.Commands.Align;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ejml.equation.Equation;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.EquationConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class StationaryAimAndShoot extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;

  //Pretty cool CTRE thing - https://api.ctr-electronics.com/phoenix6/2024-alpha/java/com/ctre/phoenix6/mechanisms/swerve/SwerveRequest.FieldCentricFacingAngle.html
  private final SwerveRequest.FieldCentricFacingAngle alignRequest = 
      new SwerveRequest.FieldCentricFacingAngle();

  private Translation2d goalLocation;
  private boolean isRed;

  public StationaryAimAndShoot(CommandSwerveDrivetrain drivetrain, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;

    addRequirements(drivetrain, shooter);

    alignRequest.HeadingController.setPID(
        AlignConstants.aimControllerP, 
        AlignConstants.aimControllerI, 
        AlignConstants.aimControllerD
    );
    alignRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
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
    shooter.setShooterRPS(targetRPS);
    // shooter.setShooterRPS(EquationConstants.quadraticRPS(distance)); //For quadratic regression formula (Just in case).

    // Find heading to goal
    double xDistToGoal = goalLocation.getX() - robotPose.getX();
    double yDistToGoal = goalLocation.getY() - robotPose.getY();
    Rotation2d goalHeading = new Rotation2d(Math.atan2(yDistToGoal, xDistToGoal));

    // Set heading
    drivetrain.setControl(alignRequest
        .withVelocityX(RobotContainer.driver.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
        .withVelocityY(RobotContainer.driver.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
        .withTargetDirection(goalHeading));

    //goals attained?
    boolean isAligned = Math.abs(robotPose.getRotation().minus(goalHeading).getDegrees()) < AlignConstants.alignToleranceDegrees;
    boolean isSpunUp = shooter.isAtVelocity(targetRPS, ShooterConstants.kRPSTolerance);

    if (isAligned && isSpunUp) {
        RobotContainer.driver.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0); //This is playstation controler and won't rumble without DS for windows
    } else {
        RobotContainer.driver.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.driver.getHID().setRumble(RumbleType.kBothRumble, 0);
    shooter.stopAllShooters();
  }
}