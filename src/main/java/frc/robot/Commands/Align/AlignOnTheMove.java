package frc.robot.Commands.Align;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

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
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.EquationConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignOnTheMove extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ProfiledPIDController thetaController;
  private final DoubleSupplier xSupplier, ySupplier;
  private final SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  public static double vGoalDist;

  public AlignOnTheMove(CommandSwerveDrivetrain drivetrain, DoubleSupplier x, DoubleSupplier y) {
    this.drivetrain = drivetrain;
    this.xSupplier = x;
    this.ySupplier = y;

    this.thetaController = new ProfiledPIDController(
        AlignConstants.aimControllerP, 
        0, // Set I to 0 as discussed
        AlignConstants.aimControllerD, 
        new TrapezoidProfile.Constraints(
            AlignConstants.alignMaxCorrectionSpeed, 
            25 // Increased acceleration for snappiness
        )
    );
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(AlignConstants.alignToleranceRadians);
    
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drivetrain.getState().Pose;
    ChassisSpeeds fieldSpeeds = drivetrain.getState().Speeds; 

    Translation2d realGoal = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red 
        ? new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY) 
        : new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);

    double distance = robotPose.getTranslation().getDistance(realGoal);

    double targetRPS = EquationConstants.calculateRPS(distance);
    //Wheel Circ 4in * RPS * slip efficiency (0.80), so just doing rotations per second times circumference to get linear velocity
    double ballSpeed = targetRPS * (0.1016 * Math.PI) * 0.80; 
    double timeOfFlight = EquationConstants.calculateTimeOfFlight(distance);//(distance / ballSpeed) + 0.5;

    Translation2d virtualGoal = new Translation2d(
        realGoal.getX() + (fieldSpeeds.vxMetersPerSecond * timeOfFlight),
        realGoal.getY() + (fieldSpeeds.vyMetersPerSecond * timeOfFlight)
    );

    vGoalDist = robotPose.getTranslation().getDistance(virtualGoal);

    double targetAngle = MathUtil.angleModulus(
        Math.atan2(virtualGoal.getY() - robotPose.getY(), virtualGoal.getX() - robotPose.getX()) + Math.PI
    );
    
    double rotVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), targetAngle);

    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double xDrive = xSupplier.getAsDouble() * maxSpeed * 0.5;
    double yDrive = ySupplier.getAsDouble() * maxSpeed * 0.5;

    drivetrain.setControl(request.withSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xDrive, 
            yDrive, 
            rotVelocity, 
            robotPose.getRotation()
        )
    ));

    AlignConstants.isAligned = thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
  }
}
