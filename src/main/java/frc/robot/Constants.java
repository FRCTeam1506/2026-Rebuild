// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Constants {

    public static class EquationConstants {
    // Equation: RPS = ax^2 + bx + c
    public static final double sA = 0;
    public static final double sB = 0;
    public static final double sC = 0;    
    
    public static double calculateRPS(double distance) {
        return sA * Math.pow(distance, 2) + sB * distance + sC;
    }
    public static double quadraticRPS (double dist) {
        return dist; //Placeholder for quadratic regression formula
    }

    public static final double tA = 0;
    public static final double tB = 0;
    public static final double tC = 0;

    public static double calculateTimeOfFlight(double distance) {
        return tA * Math.pow(distance, 2) + tB * distance + tC;
    
        }
    }

    public class FieldConstants {
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
        public static final double kRPSTolerance = 2.0;

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

    public class AlignConstants {
        public static final double alignToleranceDegrees = 2.0;
        public static final double aimControllerP = 0.05;
        public static final double aimControllerI = 0;
        public static final double aimControllerD = 0.001;
        public static final double alignMaxCorrectionSpeed = 0.5;
        public static final double alignMaxAcceleration = 0.1; 
    }

    public class PresetShots {
        public static final double closeShot = 0;
        public static final double trenchShot = 0;
        public static final double cornerShot = 0;
    }
}
