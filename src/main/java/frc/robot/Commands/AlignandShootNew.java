package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.Align.StationaryAutoAimContinuous;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AlignandShootNew extends ParallelCommandGroup {
  
    //This is a parallel command group. Keep in mind that this will not end unless it is binded to a trigger (right trigger, or wait command, etc)
    public AlignandShootNew(CommandSwerveDrivetrain drivetrain, Shooter shooter, Hopper hopper, Intake intake) {
    addCommands(
        new StationaryAutoAimContinuous(drivetrain),
        new AutoShoot(shooter, hopper),
        new InstantCommand(() -> intake.runIntake(-0.2))
    );
  }
}
