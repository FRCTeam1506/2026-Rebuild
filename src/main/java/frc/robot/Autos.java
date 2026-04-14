//everything will work. Josh worked his magic.
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

// import java.time.Instant;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.Intake.IntakeCommand;
import frc.robot.Commands.Intake.IntakeInPower;
import frc.robot.Commands.Intake.IntakeManual;
import frc.robot.Commands.Intake.IntakeOutPower;
import frc.robot.Commands.UnusedCommands.AlignandShoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Autos {
    
    static Intake intake;
    static Shooter shooter;
    static Hopper hopper;
    static CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    public Autos(CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter, Hopper hopper){
        this.intake = intake;
        this.shooter = shooter;
        this.hopper = hopper;
        this.drivetrain = drivetrain;
    }

    public void makeNamedCommands() {
        NamedCommands.registerCommand("AlignAndShoot", new AlignandShoot(drivetrain, shooter, hopper));
        //NamedCommands.registerCommand("Intake Manual", new IntakeManual(intake, 0.3));
        NamedCommands.registerCommand("Intake Manual", new InstantCommand(() -> intake.runIntake(-0.8)));
        NamedCommands.registerCommand("Intake Out", new IntakeOutPower(intake));
        NamedCommands.registerCommand("Intake In", new IntakeInPower(intake));
        NamedCommands.registerCommand("Intake Position", new IntakeCommand(intake));
    }

    public SendableChooser<Command> configureChooser(SendableChooser<Command> chooser){
        return chooser;
    }

    


}