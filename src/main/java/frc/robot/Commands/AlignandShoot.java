// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Commands.Align.StationaryAutoAim;
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

  public AlignandShoot(CommandSwerveDrivetrain drivetrain, Shooter shooter, Hopper hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.hopper = hopper;
    addCommands(new StationaryAutoAim(drivetrain),
    
        new AutoShoot(shooter, hopper)
     
     );
  }

}
