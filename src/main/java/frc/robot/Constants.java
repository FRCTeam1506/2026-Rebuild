// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class Constants {

    public static class EquationConstants {
        // Equation: RPS = ax^2 + bx + c
        public static final double sA = 2.32413;//1.41868
        public static final double sB = -6.54562;//-4.53572
        public static final double sC = 52.59076;//52.23682  //47.16806
        
        public static double calculateRPS(double distance) {
            return sA * Math.pow(distance, 2) + sB * distance + sC;
        }

        public static final double hoodA = 0.590128;//1.41868
        public static final double hoodB = -1.44757;//-4.53572
        public static final double hoodC = 0.156007;//52.23682  //47.16806
        
        
        public static double calculateHoodPassingPos(double distance) {
            return hoodA * Math.pow(distance, 2) + hoodB * distance + hoodC;
        }

        public static final double hoodPassingA = 1.49877;//1.41868
        public static final double hoodPassingB = -4.57313;//-4.53572
        public static final double hoodPassingC = 52.23682;//52.23682  //47.16806


        public static double calculateHoodPos(double distance) {
            return hoodA * Math.pow(distance, 2) + hoodB * distance + hoodC;
        }

        public static double quadraticRPS (double dist) {
            return dist; //Placeholder for quadratic regression formula
        }

        public static final double tA = 0.0553743;
        public static final double tB = -0.220291;
        public static final double tC = 1.21227;

        public static double calculateTimeOfFlight(double distance) {
            return tA * Math.pow(distance, 2) + tB * distance + tC;
        
            }
    }

    public static final class DriveConstants {
        public static final double TRENCH_MAX_VEL = 7;  // m/s
        public static final double TRENCH_MAX_ACCEL = 7.0; // m/s²
        public static final double TRENCH_MAX_ANG_VEL = 2.0; // rad/s
        public static final double TRENCH_MAX_ANG_ACCEL = 4.0; // rad/s²
    }
    
    public static class IntakeConstants {
        public static final int Intake_Motor_ID = 20;
        public static final int Intake_Lift_Motor_ID = 14;

        public static final double intakeSpeed = 0.6;
        public static final double intakeUpPosition = -0.317871;
        public static final double intakeLoweredPosition = 3.485352;
        public static final int EXTENDED_SWITCH_DIO = 0;
        public static final int RETRACTED_SWITCH_DIO = 0;
        public static boolean intakeOut;
    }

    public static class HopperConstants {
        public static final int Tower_Hopper_ID = 15;
        public static final int Tower_Hopper_Two_ID = 16;
        public static final int Hopper_Motor_ID = 25;
        public static final double hopperSpeed = .8; //0.8 //0.5
    }

    public static class HoodConstants {
        public static final double Hood_Max_Position = 5;
        public static final double Hood_Min_Position = 0;

        public static final int Hood_ID = 56;
        public static final int Hood_Limit_Switch_Port = 1;
        public static double Tuner_Hood_Pos;        
    }

    public static class ShooterConstants {
        public static final double kRPSTolerance = 1;
        public static final double kRPSTolerancePassing = 4;

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
        public static final double closeShotRPS = 45; //51  52.5
        public static final double trenchShotRPS = 56;
        public static final double cornerShotRPS = 70.75;
        public static final double passingShotRPS = 85; //90

        public static double tunerPower = 50;
    }
}
