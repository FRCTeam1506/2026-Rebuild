// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final boolean kUseLimelight = false;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        if (kUseLimelight) {
        var driveState = m_robotContainer.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        double yawRateDegPerSec = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 999999));

        LimelightHelpers.SetRobotOrientation(VisionConstants.LL_LEFT, driveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(VisionConstants.LL_RIGHT, driveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(VisionConstants.LL_BACK, driveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        
        var llMeasurement_left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LL_LEFT); //TEST THIS
        var llMeasurement_back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LL_BACK);
        var llMeasurement_right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LL_RIGHT);

        if (llMeasurement_left != null && llMeasurement_left.tagCount > 0 && Math.abs(omegaRps) < 2.0 && LimelightHelpers.getTA(VisionConstants.LL_LEFT) > 0.33)  {
            m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement_left.pose, llMeasurement_left.timestampSeconds);
        }

        if (llMeasurement_right != null && llMeasurement_right.tagCount > 0 && Math.abs(omegaRps) < 2.0 && LimelightHelpers.getTA(VisionConstants.LL_RIGHT) > 0.33) {
            m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement_right.pose, llMeasurement_right.timestampSeconds);
        }

        if (llMeasurement_back != null && llMeasurement_back.tagCount > 0 && Math.abs(omegaRps) < 2.0 && LimelightHelpers.getTA(VisionConstants.LL_BACK) > 0.33) {
            m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement_back.pose, llMeasurement_back.timestampSeconds);
        }
    }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
