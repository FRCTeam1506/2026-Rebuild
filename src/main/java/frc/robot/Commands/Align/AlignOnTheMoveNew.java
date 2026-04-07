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

public class AlignOnTheMoveNew extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ProfiledPIDController thetaController;
  private final DoubleSupplier xSupplier, ySupplier;
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

  public static double vGoalDist;

  double refinedDist;
  double refinedTOF = 0;
  Translation2d vTarget;

  public AlignOnTheMoveNew(CommandSwerveDrivetrain drivetrain, DoubleSupplier x, DoubleSupplier y) {
    this.drivetrain = drivetrain;
    this.xSupplier = x;
    this.ySupplier = y;

    this.thetaController = new ProfiledPIDController(
        AlignConstants.aimControllerP, 0, AlignConstants.aimControllerD, 
        new TrapezoidProfile.Constraints(AlignConstants.alignMaxCorrectionSpeed, 25)
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(AlignConstants.alignToleranceRadians);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    ChassisSpeeds fieldSpeeds = drivetrain.getState().Speeds;

    //latency compensation just in case, feel free to take this out
    //Should this be - sign also?? We have it below also
    double lookAhead = 0.1;
    Pose2d futurePose = new Pose2d(
        currentPose.getX() - (fieldSpeeds.vxMetersPerSecond * lookAhead), //was +
        currentPose.getY() - (fieldSpeeds.vyMetersPerSecond * lookAhead), //was +
        currentPose.getRotation().minus(Rotation2d.fromRadians(fieldSpeeds.omegaRadiansPerSecond * lookAhead)) //was .plus
    );

    Translation2d realGoal = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red 
        ? new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY) 
        : new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);

    //iterations of time of flight
    // double refinedDist = futurePose.getTranslation().getDistance(realGoal);
    // double refinedTOF = 0;
    // Translation2d vTarget = realGoal;

    for (int i = 0; i < 30; i++) {
        refinedTOF = EquationConstants.calculateTimeOfFlight(refinedDist);
        
        //should this technically be a - sign?
        vTarget = new Translation2d(
            realGoal.getX() - (fieldSpeeds.vxMetersPerSecond * refinedTOF),
            realGoal.getY() - (fieldSpeeds.vyMetersPerSecond * refinedTOF)
        );
        refinedDist = futurePose.getTranslation().getDistance(vTarget);
    }

    //final values
    vGoalDist = refinedDist;
    double targetAngle = MathUtil.angleModulus(
        Math.atan2(vTarget.getY() - futurePose.getY(), vTarget.getX() - futurePose.getX()) + Math.PI
    );
    
    double rotVelocity = thetaController.calculate(currentPose.getRotation().getRadians(), targetAngle);
    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    // drivetrain.setControl(request.withSpeeds(
    //     ChassisSpeeds.fromFieldRelativeSpeeds(
    //         -xSupplier.getAsDouble() * maxSpeed * 0.5, 
    //         -ySupplier.getAsDouble() * maxSpeed * 0.5, 
    //         rotVelocity, 
    //         currentPose.getRotation()
    //     )
    // ));
    drivetrain.setControl(request
        .withVelocityX(xSupplier.getAsDouble() * maxSpeed * 0.5)
        .withVelocityY(ySupplier.getAsDouble() * maxSpeed * 0.5)
        .withRotationalRate(rotVelocity)
    );

    AlignConstants.isAligned = thetaController.atGoal();
  }
}
