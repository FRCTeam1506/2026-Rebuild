package frc.robot.Commands.Align;

import static edu.wpi.first.units.Units.MetersPerSecond;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants.AlignConstants;
import frc.robot.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignOnTheMoveNew extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier xSupplier, ySupplier;
  
  private final SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static boolean atGoal;

  public AlignOnTheMoveNew(CommandSwerveDrivetrain drivetrain, DoubleSupplier x, DoubleSupplier y) {
    this.drivetrain = drivetrain;
    this.xSupplier = x;
    this.ySupplier = y;

    request.HeadingController.setPID(
        AlignConstants.aimControllerP, 
        AlignConstants.aimControllerI, 
        AlignConstants.aimControllerD
    );
    request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    request.HeadingController.setTolerance(AlignConstants.alignToleranceRadians);
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    atGoal = false;
    request.HeadingController.reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    
    Translation2d target = FieldConstants.vTarget;

    double targetAngle = MathUtil.angleModulus(
        Math.atan2(target.getY() - currentPose.getY(), target.getX() - currentPose.getX()) + Math.PI
    );
    // If the robot points the intake at the goal on Red side but shooter at goal on Blue (or vice versa), 
    
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        targetAngle += Math.PI; 
    }

    
    
    drivetrain.setControl(request
      .withVelocityX(xSupplier.getAsDouble() * maxSpeed * 0.6)
      .withVelocityY(ySupplier.getAsDouble() * maxSpeed * 0.6)
      .withTargetDirection(new Rotation2d(targetAngle))
      .withDeadband(0.15)
      .withRotationalDeadband(0.02) 
    );

    atGoal = request.HeadingController.atSetpoint();
  }
}
