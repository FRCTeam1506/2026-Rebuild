// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
    
    public static class IntakeConstants {
        public static final int Intake_Motor_ID = 60;
        public static final int Intake_Lift_Motor_ID = 14;
        public static final int Intake_Lift_Encoder_ID = 0;

        public static final double intakeSpeed = 0.6;
        public static final double intakeUpPosition = -0.317871;
        public static final double intakeLoweredPosition = 3.485352;
    }
    public static class HopperConstants {
        public static final int Tower_Hopper_ID = 15;
        public static final int Extender_Motor_ID = 0; //set this
        public static final int Hopper_Motor_ID = 25;
        public static final double hopperSpeed = 1; //0.5
    }
    public static class ShooterConstants {
        public static final double kRPSTolerance = 1;

        public static final int Top_Shooter_ID = 59;
        public static final int Shooter_Right_ID = 61;
        public static final int Shooter_Left_ID = 62;
        //public static final int Right_Back_Shooter_ID = 0;
    }


    public static class VisionConstants {
        public static final String LL_LEFT = "limelight-left"; //side of the turret
        public static final String LL_BACK = "limelight-back";
        public static final String LL_RIGHT = "limelight-right";
    }

    public static class AlignConstants {
        public static final double alignToleranceRadians = 0.0567;
        public static final double aimControllerP = 7; //18 //6
        public static final double aimControllerI = 0; //Consider making this 0.
        public static final double aimControllerD = 0.1; //0.2
        public static final double alignMaxCorrectionSpeed = 20; //7.5 //5.5
        public static final double alignMaxAcceleration = 40; //7, test 25 
        public static boolean isAligned;
    }

    public static class PresetShots {
        public static final double closeShotRPS = 52.5; //51
        public static final double trenchShotRPS = 56;
        public static final double cornerShotRPS = 70.75;
        public static final double passingShotRPS = 85; //90

        public static double tunerPower = 50;
    }
}
