package frc.robot.Commands.Align;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  
  //Pretty cool CTRE thing - https://api.ctr-electronics.com/phoenix6/2024-alpha/java/com/ctre/phoenix6/mechanisms/swerve/SwerveRequest.FieldCentricFacingAngle.html
  private final SwerveRequest.FieldCentricFacingAngle alignRequest = 
      new SwerveRequest.FieldCentricFacingAngle();

  private Translation2d goalLocation;
  private boolean isRed;

  public StationaryAutoAim(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    alignRequest.HeadingController.setPID(AlignConstants.aimControllerP, AlignConstants.aimControllerI, AlignConstants.aimControllerD);
    alignRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    // Determine alliance once at start to select the correct goal
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

    drivetrain.setControl(alignRequest
        .withVelocityX(RobotContainer.driver.getLeftY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
        .withVelocityY(RobotContainer.driver.getLeftX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
        .withTargetDirection(goalHeading));
    
    double angleError = Math.abs(robotPose.getRotation().minus(goalHeading).getDegrees());
    if (angleError < 2.0) {
        RobotContainer.driver.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.5);
    } else {
        RobotContainer.driver.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.driver.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
  }
}