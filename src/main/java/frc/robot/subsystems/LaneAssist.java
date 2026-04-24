// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeInPower;
import frc.robot.Constants.IntakeConstants;

public class LaneAssist extends SubsystemBase {
 
    List<Translation2d> HubZones = List.of(
        new Translation2d(FieldConstants.goalBlueX, FieldConstants.goalBlueY),
        new Translation2d(FieldConstants.goalRedX, FieldConstants.goalRedY)
    );

    //All driver perspective based on alliance
    List<Translation2d> TrenchZones = List.of(
        new Translation2d(4.231, 6.643), //Blue Left alliance
        new Translation2d(4.983, 6.622), //Blue Left neutral 

        new Translation2d(4.224, 1.429), //Blue Right alliance
        new Translation2d(5.068, 1.454), //Blue Right neutral 

        new Translation2d(12.350, 1.430), //Red Left alliance
        new Translation2d(11.510, 1.430), //Red Left neutral 

        new Translation2d(12.353, 6.610), //Red Right alliance
        new Translation2d(11.525, 6.626) //Red Right neutral 
    );

    double kP = 0.5;
    double maxDistTrench = 0.5;
    double maxDistHub = 0.5;
    Translation2d totalPush = new Translation2d(0, 0);
    double xPush, yPush;
    public static double xSpeed, ySpeed;

    //Pose2d robot;
    Pose2d currentPose;

  public LaneAssist() {
    //this.robot = robot;
  }


  @Override
  public void periodic() {
    currentPose = RobotContainer.drivetrain.getState().Pose;
    var speed = RobotContainer.drivetrain.getState().Speeds;

    xPush = 0;
    yPush = 0;

    for (Translation2d obstacal : TrenchZones) {
        double dist = currentPose.getTranslation().getDistance(obstacal);

        if (dist < maxDistTrench) {
            //Math to push robot
            xPush += (currentPose.getX() - obstacal.getX()) * kP; // + or -
            yPush += (currentPose.getY() - obstacal.getY()) * kP; // + or -
        } 
    }
    boolean tooFast = Math.abs(speed.vxMetersPerSecond) > 2 || Math.abs(speed.vyMetersPerSecond) > 2;

    if (tooFast) {
        xSpeed = (-RobotContainer.driver.getLeftY() * RobotContainer.MaxSpeed) + xPush; // + or -
        ySpeed = (-RobotContainer.driver.getLeftX() * RobotContainer.MaxSpeed) + yPush; // + or -
    } else {
        xSpeed = (-RobotContainer.driver.getLeftY() * RobotContainer.MaxSpeed);
        ySpeed = (-RobotContainer.driver.getLeftX() * RobotContainer.MaxSpeed);
    }
  }
}
