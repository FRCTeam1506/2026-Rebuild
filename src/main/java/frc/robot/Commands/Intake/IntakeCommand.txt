package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command { //hold to intake command
  private final Intake intake;

  public IntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakeLift(IntakeConstants.intakeLoweredPosition);
  }

  @Override
  public void execute() {
    if (intake.isFullyExtended()) {
      intake.runIntake(-0.6);
    } else {
      intake.runIntake(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
    intake.setIntakeLift(IntakeConstants.intakeUpPosition);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
