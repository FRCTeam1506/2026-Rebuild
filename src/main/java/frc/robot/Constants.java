// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Commands.Align.StationaryAutoAim;

/** Add your docs here. */
public class Constants {

    public static class EquationConstants {
    // Equation: RPS = ax^2 + bx + c
    public static final double sA = 1.49877;//1.41868
    public static final double sB = -4.57313;//-4.53572
    public static final double sC = 52.23682;//52.23682  //47.16806
       
    
    public static double calculateRPS(double distance) {
        return sA * Math.pow(distance, 2) + sB * distance + sC;
    }
    public static double quadraticRPS (double dist) {
        return dist; //Placeholder for quadratic regression formula
    }

    public static final double tA = 0.0153632;
    public static final double tB = 0.0744317;
    public static final double tC = 0.540794;

    public static double calculateTimeOfFlight(double distance) {
        return tA * Math.pow(distance, 2) + tB * distance + tC;
    
        }
    }

    public class FieldConstants {
        public static Translation2d goalLocation = new Translation2d(0,0);

        public static double distToGoal;
        //Positions
        //Variables for getting angle to goal.
        public static double goalRedX = 11.95; //Red Goal //12 //11.2 
        public static double goalRedY = 4; //4.034536
        public static double goalBlueX = 4.6; //Blue Goal
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

    public class IntakeConstants {
        public static final int Intake_Motor_ID = 60;
        public static final int Intake_Lift_Motor_ID = 14;
        public static final int Intake_Lift_Encoder_ID = 0;

        public static final double intakeSpeed = 0.6;
        public static final double intakeUpPose = -0.317871;
        public static final double intakeDownPose = 3.485352;
    }
    public class HopperConstants {
        public static final int Hopper_Motor_ID = 25;
        public static final double hopperSpeed = 0.5; //0.5
    }
    public class ShooterConstants {
        public static final double kRPSTolerance = 1;

        public static final int Top_Shooter_ID = 59;
        public static final int Shooter_Right_ID = 61;
        public static final int Shooter_Left_ID = 62;
        //public static final int Right_Back_Shooter_ID = 0;

        public static final int Tower_Hopper_ID = 15;

    }


    public class VisionConstants {
        public static final String LL_LEFT = "limelight-left"; //side of the turret
        public static final String LL_BACK = "limelight-back";
        public static final String LL_RIGHT = "limelight-right";
    }

    public class AlignConstants {
        public static final double alignToleranceRadians = 0.0567;
        public static final double aimControllerP = 7; //18 //6
        public static final double aimControllerI = 0; //Consider making this 0.
        public static final double aimControllerD = 0.1; //0.2
        public static final double alignMaxCorrectionSpeed = 12; //7.5 //5.5
        public static final double alignMaxAcceleration = 30; //7, test 25 
        public static boolean isAligned;
    }

    public class PresetShots {
        public static final double closeShotRPS = 52.5; //51
        public static final double trenchShotRPS = 56;
        public static final double cornerShotRPS = 70.75;
        public static final double passingShotRPS = 85; //90

        public static double tunerPower = 50;
    }
}
