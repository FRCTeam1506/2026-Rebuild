// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.Intake.IntakeCommand;
import frc.robot.Commands.Intake.OuttakeCommand;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.Commands.Shoot.ShootManual;
import frc.robot.Constants.presetShots;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final CommandPS4Controller driver = new CommandPS4Controller(0);
    public final CommandXboxController operator = new CommandXboxController(1);
    public final CommandXboxController testing = new CommandXboxController(2);


    Intake intake = new Intake();
    Shooter shooter = new Shooter();
    Hopper hopper = new Hopper();
    private boolean m_isAutoMode = false; 

    Autos autos = new Autos(drivetrain, intake, shooter, hopper);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autos.makeNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);


        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        driver.circle().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //REAL DRIVER CONTROLS:
        driver.cross().whileTrue(drivetrain.applyRequest(() -> brake));

        //Shoot
        driver.R2().whileTrue(new AutoShoot(shooter, hopper)); // Main/automatic
        driver.square().whileTrue(new ShootManual(shooter, hopper, presetShots.closeShot));
        driver.triangle().whileTrue(new ShootManual(shooter, hopper, presetShots.midShot));

        //Intake
        driver.L2().whileTrue(new IntakeCommand(intake));
        driver.L1().whileTrue(new OuttakeCommand(intake, hopper));

        //Align
        driver.R1().whileTrue(new AutoAlign(drivetrain, true));


        //Operator Controls

        //Shoot
        operator.rightTrigger().whileTrue(new AutoShoot(shooter, hopper));
        operator.a().whileTrue(new ShootManual(shooter, hopper, presetShots.closeShot));
        operator.x().whileTrue(new ShootManual(shooter, hopper, presetShots.midShot));
        operator.y().whileTrue(new ShootManual(shooter, hopper, presetShots.farShot));

        //Intake
        operator.leftTrigger().whileTrue(new IntakeCommand(intake));
        operator.leftBumper().whileTrue(new OuttakeCommand(intake, hopper));

        //Align
        operator.rightBumper().whileTrue(new AutoAlign(drivetrain, true));


        //Testing Controls
        testing.rightTrigger().whileTrue(new AutoShoot(shooter, hopper));
        testing.a().whileTrue(new ShootManual(shooter, hopper, presetShots.closeShot));
        testing.x().whileTrue(new ShootManual(shooter, hopper, presetShots.midShot));
        testing.y().whileTrue(new ShootManual(shooter, hopper, presetShots.farShot));

        //Intake
        testing.leftTrigger().whileTrue(new IntakeCommand(intake));
        testing.leftBumper().whileTrue(new OuttakeCommand(intake, hopper));
        testing.povRight().whileTrue(new InstantCommand(() -> intake.runIntake(0.5)));
        testing.povRight().onFalse(new InstantCommand(() -> intake.stopIntake()));

        //Align
        testing.rightBumper().whileTrue(new AutoAlign(drivetrain, true));

        //Hopper
        testing.leftStick().whileTrue(new InstantCommand(() -> hopper.runHopper(0.5)));
        testing.leftStick().onFalse(new InstantCommand(() -> hopper.stopHopper()));

        //Just shooter to power
        testing.povDown().whileTrue(new InstantCommand(() -> shooter.runAllShootersRPS(presetShots.closeShot)));
        testing.povDown().whileFalse(new InstantCommand(() -> shooter.stopAllShooters()));

        testing.povRight().whileTrue(new InstantCommand(() -> shooter.runAllShootersRPS(presetShots.midShot)));
        testing.povRight().whileFalse(new InstantCommand(() -> shooter.stopAllShooters()));

        testing.povUp().whileTrue(new InstantCommand(() -> shooter.runAllShootersRPS(presetShots.farShot)));
        testing.povUp().whileFalse(new InstantCommand(() -> shooter.stopAllShooters()));

        

        /* Macros:
         * MACRO SHOT:
         * driver.rightTrigger.whileTrue(new InstantCommand(() -> shooter.runAllShooters(0.5)).alongWith(new InstantCommand(() -> intake.runIntake(0.5)).alongWith(new InstantCommand(() -> hopper.runHopper(0.5)))))
         * driver.rightTrigger.onFalse(new InstantCommand(() -> shooter.stopAllShooters()).alongWith(new InstantCommand(() -> intake.stopIntake()).alongWith(new InstantCommand(() -> hopper.stopHopper()))))
         * 
         * MACRO INTAKE:
         * driver.leftTrigger.whileTrue(new InstantCommand(() -> intake.runIntake(0.5)).alongWith(new InstantCommand(() -> intake.runIntakeLift(0.5))))
         * driver.leftTrigger.onFalse(new InstantCommand(() -> intake.stopIntake()).alongWith(new InstantCommand(() -> intake.stopIntakeLift())))
         */
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    
}
