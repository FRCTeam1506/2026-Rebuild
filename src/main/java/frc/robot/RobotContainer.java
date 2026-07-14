// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
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
import frc.robot.Commands.Align.AlignOnTheMoveNew;
import frc.robot.Commands.AutoDrive.DriveToTrench;
import frc.robot.Commands.AutoPathing.DriveShortestPath;
import frc.robot.Commands.AutoPathing.DriveToCross;
import frc.robot.Commands.AutoPathing.Pathing;
import frc.robot.Commands.Intake.IntakeInPower;
import frc.robot.Commands.Intake.IntakeManual;
import frc.robot.Commands.Intake.IntakeOutPower;
import frc.robot.Commands.Intake.JitterIntake;
import frc.robot.Commands.Intake.Outtake;
import frc.robot.Commands.Macros.AlignandShootStationary;
import frc.robot.Commands.Macros.SOTM;
import frc.robot.Commands.Shoot.AutoSOTM;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.Commands.Shoot.ManualShoot;
import frc.robot.Commands.Shoot.TunerShot;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.PresetShots;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.8).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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

    public  final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);
    public final CommandXboxController testing = new CommandXboxController(2);


    Intake intake = new Intake();
    Shooter shooter = new Shooter(drivetrain, intake);
    Hopper hopper = new Hopper();
    Pathing pathing = new Pathing(drivetrain);
    Hood hood = new Hood();
    private boolean m_isAutoMode = false; 

    Autos autos = new Autos(drivetrain, intake, shooter, hopper, hood);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autos.makeNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Attach the sim-only intake to the drivetrain's physics body so it can collect Fuel.
        if (Utils.isSimulation()) {
            intake.initSim(drivetrain.getMapleSimDrive());
        }

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
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-testing.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-testing.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-testing.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        //DRIVER CONTROLS:
        // driver.R1().whileTrue(
        //     new AlignOnTheMoveNew(
        //         drivetrain,
        //         () -> -driver.getLeftY(),
        //         () -> -driver.getLeftX()
        //     )
        // );



        driver.rightTrigger().whileTrue(
            new SOTM(
                drivetrain,
                shooter,
                hopper,
                intake,
                hood,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX()
            )
        );
        // driver.rightTrigger().whileTrue(
        //     new AlignOnTheMoveNew(
        //         drivetrain,
        //         () -> -driver.getLeftY(),
        //         () -> -driver.getLeftX()
        //     )
        // );



        //driver.R2().whileTrue(new JitterIntake(intake).repeatedly().unless(operator.leftTrigger()).unless(driver.L2()));
        driver.rightTrigger().whileFalse(new InstantCommand(() -> intake.stopIntakeLift())).onFalse(new InstantCommand(() -> intake.runIntake(0)));      
          

        driver.rightBumper().whileFalse(new InstantCommand(() -> intake.stopIntakeLift())).onFalse(new InstantCommand(() -> intake.runIntake(0)));        
        // Reset the field-centric heading on left CIRCLE (David likes circle) press.
        driver.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //Brake on cross, x wheel position
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

        //Shoot
        //driver.R2().whileTrue(new AutoShoot(shooter, hopper)); //Old AlignandShoot Command Sequential Group
        driver.rightBumper().whileTrue(new AlignandShootStationary(drivetrain, shooter, hopper, intake, hood)).onFalse(new InstantCommand(() -> intake.stopIntake())); //Parallel Command Group, align and Shoot, ends on trigger
        driver.rightBumper().whileFalse(new InstantCommand(() -> intake.stopIntakeLift())).onFalse(new InstantCommand(() -> intake.runIntake(0)));        

        driver.x().whileTrue(new ManualShoot(shooter, hopper, hood, PresetShots.closeShotRPS)); //Tower shot
        driver.x().whileTrue(new JitterIntake(intake).repeatedly()); //Tower shot
        driver.x().whileFalse(new InstantCommand(() -> intake.stopIntakeLift())); //Tower shot
        driver.y().whileTrue(new ManualShoot(shooter, hopper, hood, PresetShots.cornerShotRPS)); //Corner Shot
        //driver.povUp().whileTrue(new ManualShoot(shooter, hopper, PresetShots.passingShotRPS)); //Passing Shot

        //Intake    
        //().whileTrue(new IntakeOutNew(intake)).onFalse(new IntakeInNew(intake));
        driver.leftTrigger().whileTrue(new InstantCommand(() -> intake.runIntake(-0.9)));
        driver.leftTrigger().onTrue(new IntakeOutPower(intake));
        driver.leftTrigger().whileFalse(new InstantCommand(() -> intake.runIntake(0)));
        driver.povDown().onTrue(new IntakeOutPower(intake));
        driver.povUp().onTrue(new IntakeInPower(intake));
        //driver.povRight().onTrue(new IntakeToggle(intake));

        //driver.L1().whileTrue(new OuttakeCommand(intake, hopper)); //PUT THIS BACK IN
        //driver.L1().whileTrue(new AutoSOTMNew(shooter, hopper));
        
        driver.leftBumper().whileTrue(new DriveShortestPath(drivetrain, pathing));


        //Stationary Align:
        //driver.R1().whileTrue(new StationaryAutoAim(drivetrain));

        //Tuner Shot:
        // driver.povUp().onTrue(new InstantCommand(() -> shooter.upPower()));
        // driver.povDown().onTrue(new InstantCommand(() -> shooter.downPower()));
        // driver.povRight().whileTrue(new TunerShoot(shooter, hopper));

        

        //OPERATOR CONTROLS:
        //Shoot only on right trigger for operator:
        // operator.rightTrigger().whileTrue(new AutoShoot(shooter, hopper));
        operator.rightTrigger().whileTrue(new AlignandShootStationary(drivetrain, shooter, hopper, intake, hood)).onFalse(new InstantCommand(() -> intake.stopIntake())); //Parallel Command Group, align and Shoot, ends on trigger
        operator.rightTrigger().whileFalse(new InstantCommand(() -> intake.stopIntakeLift())).onFalse(new InstantCommand(() -> intake.runIntake(0)));        

        //Run shooter and hopper in reverse:
        operator.b().whileTrue(new InstantCommand(() -> shooter.runAllShootersSpeed(-0.25)).alongWith(new InstantCommand(() -> hopper.runHopper(-0.5))));
        
        //Run shooter and hopper in reverse:
        operator.b().whileFalse(new InstantCommand(() -> shooter.runAllShootersSpeed(0)).alongWith(new InstantCommand(() -> hopper.runHopper(0))));

        //Preset Shots:
        operator.a().whileTrue(new ManualShoot(shooter, hopper, hood, PresetShots.closeShotRPS));
        operator.x().whileTrue(new ManualShoot(shooter, hopper, hood, PresetShots.cornerShotRPS));
        operator.x().whileTrue(new JitterIntake(intake).repeatedly()); //Tower shot
        operator.x().whileFalse(new InstantCommand(() -> intake.stopIntakeLift())); //Tower shot
        operator.y().whileTrue(new ManualShoot(shooter, hopper, hood, PresetShots.passingShotRPS));

        //SOTM Auto Shoot:
        //operator.rightBumper().whileTrue(new AutoSOTM(shooter, hopper));
        // operator.rightBumper().whileTrue(
        //     new AutoSOTM(
        //         shooter,
        //         hopper
        //     )
        // );
        //operator.rightBumper().whileTrue(new JitterIntake(intake).repeatedly().unless(operator.leftTrigger()).unless(driver.leftTrigger()));
        operator.rightBumper().whileTrue(new AutoSOTM(shooter, hopper, hood).alongWith(new JitterIntake(intake)).repeatedly());
        operator.rightBumper().whileFalse(new InstantCommand(() -> intake.stopIntakeLift())).onFalse(new InstantCommand(() -> intake.runIntake(0)));        


        //Bring this back in: Rev Hopper Only
        //operator.rightBumper().whileTrue(new InstantCommand(() -> hopper.runHopper(-HopperConstants.hopperSpeed)));
        //operator.rightBumper().whileFalse(new InstantCommand(() -> hopper.stopHopper()));

        //Intake
        //operator.leftTrigger().whileTrue(new IntakeOutNew(intake)).onFalse(new IntakeInNew(intake));
        //operator.leftTrigger().whileTrue(new InstantCommand(() -> intake.runIntake(-0.8))).onFalse(new InstantCommand(() -> intake.runIntake(0)));
        operator.leftTrigger().whileTrue(new InstantCommand(() -> intake.runIntake(-0.9)));
        operator.leftTrigger().onTrue(new IntakeOutPower(intake));
        operator.leftTrigger().whileFalse(new InstantCommand(() -> intake.runIntake(0)));
        operator.povDown().onTrue(new IntakeInPower(intake));
        operator.povUp().onTrue(new IntakeOutPower(intake));

        //Jitter Intake:
        // operator.povUp().whileTrue(new JitterIntake(intake).repeatedly());
        // operator.povUp().whileFalse(new IntakeInPower(intake).alongWith(new InstantCommand(() -> intake.runIntake(0))));//Check this!

        //Outtake:
        operator.leftBumper().whileTrue(new Outtake(intake, hopper)); //OLD INTAKE!!

        //Align
        //Aim while holding the right bumper
        //operator.rightBumper().whileTrue(new StationaryAutoAim(drivetrain));
        
        //Align and Shoot for operator needed:
        //operator.rightTrigger().whileTrue(new AlignandShoot(drivetrain, shooter, hopper));
        //operator.rightBumper().whileFalse(new InstantCommand(() -> shooter.stopAllShooters()).alongWith(new InstantCommand(() -> hopper.stopHopper())));


        
        //TESTING CONTROLS:
        testing.rightTrigger().whileTrue(new ManualShoot(shooter, hopper, hood, 50));
        testing.rightBumper().whileTrue(new TunerShot(shooter, hopper, hood, PresetShots.tunerPower, HoodConstants.Tuner_Hood_Pos));
        // testing.a().whileTrue(new ManualShoot(shooter, hopper, hood, PresetShots.closeShotRPS));
        // testing.x().whileTrue(new ManualShoot(shooter, hopper, hood, PresetShots.trenchShotRPS));
        // testing.y().whileTrue(new ManualShoot(shooter, hopper, hood, PresetShots.cornerShotRPS));

        //Intake
        //testing.leftTrigger().whileTrue(new IntakeCommand(intake));
        testing.leftBumper().whileTrue(new Outtake(intake, hopper));
        testing.leftTrigger().whileTrue(new InstantCommand(() -> intake.runIntake(-0.9)));
        testing.leftTrigger().onTrue(new IntakeOutPower(intake));
        testing.leftTrigger().whileFalse(new InstantCommand(() -> intake.runIntake(0)));
        // testing.povRight().whileTrue(new InstantCommand(() -> intake.runIntake(0.5)));
        // testing.povRight().onFalse(new InstantCommand(() -> intake.stopIntake()));

        //Align
        //testing.rightBumper().whileTrue(new MovingAutoAim(drivetrain, true));

        //Hopper
        testing.leftStick().whileTrue(new InstantCommand(() -> hopper.runHopper(0.5)));
        testing.leftStick().onFalse(new InstantCommand(() -> hopper.stopHopper()));

        //Tuning
        //testing.povUp().onTrue(new InstantCommand(() -> shooter.upPower()));
        //testing.povDown().onTrue(new InstantCommand(() -> shooter.downPower()));
        testing.y().whileTrue(new InstantCommand(() -> hood.hoodUpPower()));
        testing.y().whileFalse(new InstantCommand(() -> hood.stopHood()));
        testing.a().whileTrue(new InstantCommand(() -> hood.hoodDownPower()));
        testing.a().whileFalse(new InstantCommand(() -> hood.stopHood()));
        testing.povRight().onTrue(new InstantCommand(() -> hood.hoodUp()));
        testing.povLeft().onTrue(new InstantCommand(() -> hood.hoodDown()));
        testing.povUp().onTrue(new InstantCommand(() -> shooter.upPower()));
        testing.povDown().onTrue(new InstantCommand(() -> shooter.downPower()));

    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
    
    
}
