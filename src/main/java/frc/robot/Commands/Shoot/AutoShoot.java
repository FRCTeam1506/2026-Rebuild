// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EquationConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  Shooter shooter;
  Hopper hopper;

  /** Creates a new Shoot. */
  public AutoShoot(Shooter shooter, Hopper hopper) {
    this.shooter = shooter;
    this.hopper = hopper;
    addRequirements(shooter, hopper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterRPS(EquationConstants.calculateRPS(FieldConstants.distToGoal));
    if (shooter.isAtVelocity(EquationConstants.calculateRPS(FieldConstants.distToGoal), ShooterConstants.kRPSTolerance)) {
      hopper.runHopper(HopperConstants.hopperSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAllShooters();
    hopper.stopHopper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
