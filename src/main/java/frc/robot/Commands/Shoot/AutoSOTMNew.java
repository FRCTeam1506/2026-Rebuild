package frc.robot.Commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Align.AlignOnTheMoveNew;
import frc.robot.Constants.EquationConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class AutoSOTMNew extends Command {
  private final Shooter shooter;
  private final Hopper hopper;

  public AutoSOTMNew(Shooter shooter, Hopper hopper) {
    this.shooter = shooter;
    this.hopper = hopper;
    addRequirements(shooter, hopper);
  }

  @Override
  public void execute() {
    // Rev the shooter based on the final converged virtual distance
    double targetRPS = EquationConstants.calculateRPS(AlignOnTheMoveNew.vGoalDist);
    shooter.setShooterRPS(targetRPS);

    if (shooter.isAtVelocity(targetRPS, ShooterConstants.kRPSTolerance)) {
        hopper.runHopper(-HopperConstants.hopperSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopAllShooters();
    hopper.stopHopper();
  }

  @Override
  public boolean isFinished() { return false; }
}
