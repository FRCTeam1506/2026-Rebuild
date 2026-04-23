// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final boolean kUseLimelight = true;

    private final edu.wpi.first.math.filter.LinearFilter xFilter = edu.wpi.first.math.filter.LinearFilter.movingAverage(5);
    private final edu.wpi.first.math.filter.LinearFilter yFilter = edu.wpi.first.math.filter.LinearFilter.movingAverage(5);



    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        //SmartDashboard.putNumber("auto align heading", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
        //SmartDashboard.putNumber("heading", );


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
        double pigeonYaw = m_robotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        double yawRateDegPerSec = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);
        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 999999)); //consider 0.15

        SmartDashboard.putNumber("Pigeon Yaw LL", pigeonYaw);
        SmartDashboard.putNumber("Pose Yaw LL", headingDeg);

        // if(headingDeg > 0){
        //     headingDeg -= 180;
        // }
        // else{
        //     headingDeg += 180;
        // }

        LimelightHelpers.SetRobotOrientation(VisionConstants.LL_LEFT, headingDeg, 0, 0, 0, 0, 0); 
        var llMeasurement_left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LL_LEFT); //TEST THIS //mt2

        LimelightHelpers.SetRobotOrientation(VisionConstants.LL_RIGHT, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement_right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LL_RIGHT); //mt2

        LimelightHelpers.SetRobotOrientation(VisionConstants.LL_BACK, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement_back = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LL_BACK);

        if (llMeasurement_left != null && llMeasurement_left.tagCount > 0 && Math.abs(omegaRps) < 2.0 && LimelightHelpers.getTA(VisionConstants.LL_LEFT) > 0.1)  { //0.1 //&& LimelightHelpers.getTA(VisionConstants.LL_LEFT) > 0.33
            m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement_left.pose, llMeasurement_left.timestampSeconds);
        }

        if (llMeasurement_right != null && llMeasurement_right.tagCount > 0 && Math.abs(omegaRps) < 2.0 && LimelightHelpers.getTA(VisionConstants.LL_RIGHT) > 0.1) { //0.1
            m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement_right.pose, llMeasurement_right.timestampSeconds);
        }

        if (llMeasurement_back != null && llMeasurement_back.tagCount > 0 && Math.abs(omegaRps) < 2.0 && LimelightHelpers.getTA(VisionConstants.LL_BACK) > 0.1) {
            m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement_back.pose, llMeasurement_back.timestampSeconds);
        }
    }

    var state = m_robotContainer.drivetrain.getState();
    
    //consider this to help smooth out speeds
    ChassisSpeeds rawFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());

    double smoothVX = xFilter.calculate(rawFieldSpeeds.vxMetersPerSecond);
    double smoothVY = yFilter.calculate(rawFieldSpeeds.vyMetersPerSecond);
    ChassisSpeeds filteredSpeeds = new ChassisSpeeds(smoothVX, smoothVY, rawFieldSpeeds.omegaRadiansPerSecond);

    FieldConstants.updateMovingTarget(state.Pose, filteredSpeeds);
    
    FieldConstants.updateActiveGoal(m_robotContainer.drivetrain.getState().Pose);

    //FieldConstants.updateMovingTarget(state.Pose, ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation()));


    SmartDashboard.putString("Zone/Current", FieldConstants.currentZone.toString());
    SmartDashboard.putBoolean("Zone/isHub", FieldConstants.currentZone == FieldConstants.FieldZone.HUB);
    SmartDashboard.putBoolean("Zone/isMailing", 
        FieldConstants.currentZone == FieldConstants.FieldZone.MAILING_LEFT || 
        FieldConstants.currentZone == FieldConstants.FieldZone.MAILING_RIGHT);
    
    SmartDashboard.putNumber("Zone/DistToActiveGoal", FieldConstants.distToGoal);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        //FieldConstants.allianceColor = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
        // Not needed: goal locattion is updated in shooter periodic
        //boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
        //  FieldConstants.goalLocation = isRed ? 
        //     new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY) : 
        //     new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);
        
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
    //FieldConstants.allianceColor = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    //boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
        // FieldConstants.goalLocation = isRed ? 
        // new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY) : 
        // new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY);

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {

    }

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
