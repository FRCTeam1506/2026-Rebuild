package frc.robot.Commands.Macros;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.Align.AlignOnTheMoveNew;
import frc.robot.Commands.Align.StationaryAutoAimContinuous;
import frc.robot.Commands.Intake.JitterIntake;
import frc.robot.Commands.Shoot.AutoSOTM;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.Commands.UnusedCommands.AlignOnTheMove;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SOTM extends ParallelCommandGroup {
    //This is a parallel command group. Keep in mind that this will not end unless it is binded to a trigger (right trigger, or wait command, etc)
    public SOTM(CommandSwerveDrivetrain drivetrain, Shooter shooter, Hopper hopper, Intake intake, DoubleSupplier x, DoubleSupplier y) {
      
    addCommands(
        new AlignOnTheMoveNew(drivetrain, x, y),
        new AutoSOTM(shooter, hopper),//,
        //new InstantCommand(() -> intake.runIntake(-0.2))
        new JitterIntake(intake).repeatedly()

    );
  }
}
