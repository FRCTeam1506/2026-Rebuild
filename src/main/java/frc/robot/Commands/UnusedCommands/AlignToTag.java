// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.UnusedCommands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command {
  private final CommandSwerveDrivetrain drivetrain;

    private final ProfiledPIDController thetaController;

    private final SwerveRequest.FieldCentric alignRequest;

    double rotationalVelocity;
    double radians;
  
  /** Creates a new AutoAlign. */
    public AlignToTag(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        alignRequest = new SwerveRequest.FieldCentric();

        thetaController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(2, 2));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    rotationalVelocity = thetaController.calculate(LimelightHelpers.getTX(VisionConstants.LL_BACK), 0);
    radians = Math.toRadians(rotationalVelocity);


    drivetrain.setControl(alignRequest
            //.withVelocityX(RobotContainer.driver.getLeftY())//forwards and backwards? YES
            //.withVelocityY(RobotContainer.driver.getLeftX())  reaching into robot container is bad practice
            .withRotationalRate(radians));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    drivetrain.setControl(alignRequest
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
    );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
