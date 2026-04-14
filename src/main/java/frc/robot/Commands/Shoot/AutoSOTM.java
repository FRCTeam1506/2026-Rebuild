package frc.robot.Commands.Shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Align.AlignOnTheMoveNew;
import frc.robot.Constants.EquationConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.FieldZone;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class AutoSOTM extends Command {
  private final Shooter shooter;
  private final Hopper hopper;

  public AutoSOTM(Shooter shooter, Hopper hopper) {
    this.shooter = shooter;
    this.hopper = hopper;
    addRequirements(shooter, hopper);
  }

  @Override
  public void execute() {
    double targetRPS = EquationConstants.calculateRPS(FieldConstants.vGoalDist);
    
    if (FieldConstants.currentZone == FieldZone.MAILING_LEFT || 
        FieldConstants.currentZone == FieldZone.MAILING_RIGHT) {
        targetRPS -= 20;
    }
    else {
      targetRPS += 2;
    }
    if (targetRPS > 90) {
      targetRPS = 90;
    }
    shooter.setShooterRPS(targetRPS);
    
    //hopper
    if (shooter.isAtVelocity(targetRPS, ShooterConstants.kRPSTolerance) && AlignOnTheMoveNew.atGoal == true) {
        hopper.runHopper(HopperConstants.hopperSpeed);
    }
  
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopAllShooters();
    hopper.stopHopper();
  }

  @Override
  public boolean isFinished() { 
    return false; 
  }
}
