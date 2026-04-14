package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeConditional extends ConditionalCommand { //tap for intake, tap again to pull up
  public IntakeConditional(Intake intake) {
    super(
      //If extended, pull it in
      new InstantCommand(() -> intake.raiseIntake(), intake),
      
      //If not extended, push it out
      new InstantCommand(() -> intake.lowerIntake(), intake),
      
      //condition
      () -> intake.isFullyExtended()
    );
  }
}
