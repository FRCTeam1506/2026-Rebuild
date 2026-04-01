// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.EquationConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Commands.Align.StationaryAutoAim;
import frc.robot.Commands.Align.StationaryAutoAimEnd;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignandShoot extends SequentialCommandGroup {
  /** Creates a new AlignandShoot. */
  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;
  private final Hopper hopper;
  //Pose2d robotPose = drivetrain.getState().Pose;

  // boolean isAligned = Math.abs((robotPose.getRotation().getDegrees() + 180) - AlignConstants.goalHeading) <= AlignConstants.alignToleranceDegrees;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public AlignandShoot(CommandSwerveDrivetrain drivetrain, Shooter shooter, Hopper hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.hopper = hopper;

    Command brakeCommand = drivetrain.applyRequest(() -> brake);
    addCommands(
        new InstantCommand(() -> shooter.setShooterRPS(EquationConstants.calculateRPS(FieldConstants.distToGoal) + 0.5)), //Rev up the shooter before aligning
        //new StationaryAutoAim(drivetrain).withTimeout(3),
        new StationaryAutoAim(drivetrain),
        //brakeCommand.withTimeout(0.2),
        new ParallelCommandGroup(
          new StationaryAutoAimEnd(drivetrain),
          new AutoShoot(shooter, hopper)
        )
    );
  }

}
