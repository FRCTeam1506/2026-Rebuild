package frc.robot.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopAutoAim extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ProfiledPIDController thetaController;
  private final DoubleSupplier xSupplier, ySupplier;
  private final SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds();

  public TeleopAutoAim(CommandSwerveDrivetrain drivetrain, DoubleSupplier x, DoubleSupplier y) {
    this.drivetrain = drivetrain;
    this.xSupplier = x;
    this.ySupplier = y;
    
    this.thetaController = new ProfiledPIDController(
        AlignConstants.aimControllerP, 0, AlignConstants.aimControllerD, 
        new TrapezoidProfile.Constraints(AlignConstants.alignMaxCorrectionSpeed, 25) // High acceleration!
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drivetrain.getState().Pose;
    
    Translation2d goal = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red 
        ? new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY) 
        : new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);

    double targetAngle = MathUtil.angleModulus(Math.atan2(goal.getY() - robotPose.getY(), goal.getX() - robotPose.getX()) + Math.PI);
    double rotVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), targetAngle);

    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    double xSpeed = xSupplier.getAsDouble() * maxSpeed;
    double ySpeed = ySupplier.getAsDouble() * maxSpeed;


    drivetrain.setControl(request.withSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotVelocity, robotPose.getRotation())
    ));

    AlignConstants.isAligned = thetaController.atGoal();
  }
}
