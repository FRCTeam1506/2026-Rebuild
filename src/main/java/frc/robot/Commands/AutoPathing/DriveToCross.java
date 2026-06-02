// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoPathing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToCross extends Command {
  CommandSwerveDrivetrain drivetrain;
  Pathing pathing;
  Command activePathCommand;

  PathConstraints constraints;

  /** Creates a new driveToShortest. */
  public DriveToCross(CommandSwerveDrivetrain drivetrain, Pathing pathing) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.pathing = pathing;

    this.constraints = new PathConstraints(
        Pathing.trenchPassSpeed, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d targetPose = pathing.shortestPose(drivetrain);

    activePathCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);

    activePathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (activePathCommand != null) {
        activePathCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (activePathCommand != null) {
        activePathCommand.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (activePathCommand != null) {
        return activePathCommand.isFinished();
    }
    return true;
  }
}
