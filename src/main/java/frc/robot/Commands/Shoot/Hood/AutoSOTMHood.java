package frc.robot.Commands.Shoot.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Align.AlignOnTheMoveNew;
import frc.robot.Constants.EquationConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.FieldZone;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class AutoSOTMHood extends Command {
  private final Shooter shooter;
  private final Hopper hopper;
  private final Hood hood;

  public AutoSOTMHood(Shooter shooter, Hopper hopper, Hood hood) {
    this.shooter = shooter;
    this.hopper = hopper;
    this.hood = hood;
    addRequirements(shooter, hopper, hood);
  }

  @Override
  public void execute() {
    double targetRPS = EquationConstants.calculateRPS(FieldConstants.vGoalDist);
    double hoodPose = EquationConstants.calculateRPS(FieldConstants.vGoalDist);
    // if (FieldConstants.currentZone == FieldZone.MAILING_LEFT || 
    //     FieldConstants.currentZone == FieldZone.MAILING_RIGHT) {
    //     targetRPS -= 20;
    // }
    // else {
    //   targetRPS += 2;
    // }
    // if (targetRPS > 90) {
    //   targetRPS = 90;
    // }
    shooter.setShooterRPS(targetRPS);
    hood.moveHood(hoodPose);
    
    //hopper
    //Check if we are aligned
    // if (shooter.isAtVelocity(targetRPS, ShooterConstants.kRPSTolerance) && AlignOnTheMoveNew.atGoal == true) {
    //     hopper.runHopper(HopperConstants.hopperSpeed);
    // }
    if (shooter.isAtVelocity(targetRPS, ShooterConstants.kRPSTolerance)) {
        hopper.runHopper(HopperConstants.hopperSpeed);
    }
  
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopAllShooters();
    hopper.stopHopper();
    //hood.moveHood(HoodConstants.Hood_Min_Position);
  }

  @Override
  public boolean isFinished() { 
    return false; 
  }
}
