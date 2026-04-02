package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.Align.StationaryAutoAimEnd;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class AlignandShootNew extends ParallelCommandGroup {
  
  public AlignandShootNew(CommandSwerveDrivetrain drivetrain, Shooter shooter, Hopper hopper) {
    addCommands(
        new StationaryAutoAimEnd(drivetrain),
        new AutoShoot(shooter, hopper)
    );
  }
}
