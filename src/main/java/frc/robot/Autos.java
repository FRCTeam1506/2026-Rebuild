//everything will work. Josh worked his magic.
package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Commands.Intake.IntakeInPower;
import frc.robot.Commands.Intake.IntakeManual;
import frc.robot.Commands.Intake.IntakeOutPower;
import frc.robot.Commands.Intake.JitterIntake;
import frc.robot.Commands.Macros.AlignandShootStationary;
import frc.robot.Commands.UnusedCommands.AlignandShoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Autos {
    
    private final Intake intake;
    private final Shooter shooter;
    private final Hopper hopper;
    private final CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    public Autos(CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter, Hopper hopper){
        this.intake = intake;
        this.shooter = shooter;
        this.hopper = hopper;
        this.drivetrain = drivetrain;
    }

    public void makeNamedCommands() {
        NamedCommands.registerCommand("AlignAndShoot", new AlignandShootStationary(drivetrain, shooter, hopper, intake));
        NamedCommands.registerCommand("Intake Manual", new InstantCommand(() -> intake.runIntake(-0.9)));
        NamedCommands.registerCommand("Intake Out", new IntakeOutPower(intake));
        //NamedCommands.registerCommand("Intake In", new IntakeInPower(intake));
        // startEnd factory, backed by the StartEndCommand (Java, C++, Python) class, calls one lambda when scheduled, and then a second lambda when interrupted. https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html
        NamedCommands.registerCommand("Intake End", 
            new StartEndCommand(() -> intake.runIntake(-0.9), 
            () -> intake.runIntake(0), 
            intake));
            }

    public SendableChooser<Command> configureChooser(SendableChooser<Command> chooser){
        return chooser;
    }

    


}