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
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.EquationConstants;
import frc.robot.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignOnTheMoveNew extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ProfiledPIDController thetaController;
  private final DoubleSupplier xSupplier, ySupplier;
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

  public static double vGoalDist;
  double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static boolean atGoal;

  public AlignOnTheMoveNew(CommandSwerveDrivetrain drivetrain, DoubleSupplier x, DoubleSupplier y) {
    this.drivetrain = drivetrain;
    this.xSupplier = x;
    this.ySupplier = y;

    this.thetaController = new ProfiledPIDController(
        AlignConstants.aimControllerP, 0, AlignConstants.aimControllerD, 
        new TrapezoidProfile.Constraints(AlignConstants.alignMaxCorrectionSpeed, 25)
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(AlignConstants.alignToleranceRadians); // was * 2
    
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    
    Translation2d target = FieldConstants.vTarget;

    double targetAngle = MathUtil.angleModulus(
        Math.atan2(target.getY() - currentPose.getY(), target.getX() - currentPose.getX()) + Math.PI
    );
    
    double rotVelocity = thetaController.calculate(currentPose.getRotation().getRadians(), targetAngle);
    
    drivetrain.setControl(request
      .withVelocityX(xSupplier.getAsDouble() * maxSpeed * 0.6)
      .withVelocityY(ySupplier.getAsDouble() * maxSpeed * 0.6)
      .withRotationalRate(rotVelocity)
      .withDeadband(0.15) 
      .withRotationalDeadband(0.02) 
  );

    atGoal = thetaController.atGoal();
  
  }
}

