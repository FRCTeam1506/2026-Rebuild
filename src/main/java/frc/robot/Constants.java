// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Constants {
    public class fieldConstants {
        public static double distToGoal;
        //Positions
        //Variables for getting angle to goal.
        public static double goalRedX = 12; //Red Goal
        public static double goalRedY = 4.034536;
        public static double goalBlueX = -12; //Blue Goal
        public static double goalBlueY = 4.034536;

        //FIELD LOCATIONS:
        public static final double redLine = 11.5; //used to be 12.6, made it 11.5 for more accurate zone of when we want to do mailing funciton
        public static final double middleY = 4;
        public static final double blueLine = 4.1;

        public static final double goalRightRedY = 7;//7.211
        public static final double goalRightRedX = 15;

        public static final double goalLeftRedY = 1.5; //0.8
        public static final double goalLeftRedX = 15;

        public static final double goalRightBlueY = 1.2; //7.756
        public static final double goalRightBlueX = 2;

        public static final double goalLeftBlueY = 7; //3
        public static final double goalLeftBlueX = 1.5;
    }

    public class alignCalculations{
        public static double theta;
        public static double thetaX, thetaY;
        public static double toDegree;
        public static double preHeading;
        public static double wantedHeading;
        public static double dist;
        public static Translation2d targetVec;
        public static double distToGoal;

        //Robot Poses
        public static Pose2d robotPose;
        public static double robotPoseX;
        public static double robotPoseY;
        public static double omega;
        public static double heading;

        public static Translation2d rotationalVelocityField;

        //Virtual Robot Poses
        public static double vRobotY;
        public static double vRobotX;
        public static double vRotationalRobotY;
        public static double vRotationalRobotX;
        public static double totalFieldVy;
        public static double totalFieldVx;
        public static Translation2d vRobotPose;
    }
    public class IntakeConstants {
        public static final int Intake_Motor_ID = 0;
        public static final int Intake_Lift_Motor_ID = 0;
        public static final int Intake_Lift_Encoder_ID = 0;

        public static final double intakeSpeed = 0.6;
        public static final int intakeUpPose = 0;
        public static final int intakeDownPose = 0;
    }
    public class HopperConstants {
        public static final int Hopper_Motor_ID = 0;
        public static final double hopperSpeed = 0;
    }
    public class ShooterConstants {
        public static final int Left_Front_Shooter_ID = 0;
        public static final int Right_Front_Shooter_ID = 0;
        public static final int Left_Back_Shooter_ID = 0;
        public static final int Right_Back_Shooter_ID = 0;

        //TESTING:
        public static final int Left_Shooter_ID = 0;
        public static final int Right_Shooter_ID = 0;
    }


    public class VisionConstants {
        public static final String LL_LEFT = "limelight-left"; //side of the turret
        public static final String LL_BACK = "limelight-back";
        public static final String LL_RIGHT = "limelight-right";
    }

    public class presetShots {
        public static final double closeShot = 0;
        public static final double midShot = 0;
        public static final double farShot = 0;
    }
}
