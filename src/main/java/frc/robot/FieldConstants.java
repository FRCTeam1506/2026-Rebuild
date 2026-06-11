package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.EquationConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FieldConstants {
    // Current State
    public static Translation2d goalLocation = new Translation2d(0,0);
    public static double distToGoal;
    public static boolean passing;

    //public static Pose2d wantedPose;
    public static Pose2d finalPose;
    
    public enum FieldZone {
        HUB, MAILING_LEFT, MAILING_RIGHT, UNKNOWN
    }
    
    public static FieldZone currentZone = FieldZone.UNKNOWN;

    public static double vGoalDist;
    public static Translation2d vTarget = new Translation2d();

    // Standard Hub Goals
    public static final double goalRedX = 11.95;
    public static final double goalRedY = 4.0;
    public static final double goalBlueX = 4.6;
    public static final double goalBlueY = 4.034;

    // Zone Thresholds
    public static final double redLine = 11.5;
    public static final double blueLine = 4.1;
    public static final double middleY = 4.024;

    // Mailing Goals
    public static final double goalRightRedX = 15.0;
    public static final double goalRightRedY = 7.0;
    public static final double goalLeftRedX = 15.0;
    public static final double goalLeftRedY = 1.5;

    public static final double goalRightBlueX = 2.0;
    public static final double goalRightBlueY = 1.2;
    public static final double goalLeftBlueX = 1.5;
    public static final double goalLeftBlueY = 7.0;

    public static final double BLUE_LEFT_TRENCH_Y = 7.430;
    public static final double BLUE_RIGHT_TRENCH_Y = 0.600;
    public static final double BLUE_TRENCH_X_ENTRY = 6.187;
    public static final double BLUE_TRENCH_X_EXIT = 2.875;

    public static final double RED_LEFT_TRENCH_Y = 0.800;   
    public static final double RED_RIGHT_TRENCH_Y = 7.0;  
    public static final double RED_TRENCH_X_ENTRY = 11.30; 
    public static final double RED_TRENCH_X_EXIT = 13.575;

    
    public static final double LINEUP_OFFSET_METERS = 0.5;

    boolean isRedAlliance;

    // For Pathing
    // public static int crossingZone; //1 = Left Trench, 2 = Left Bump, 3 = Right Bump, 4 = Right Trench
    // public static Pose2d leftBump, leftTrench, rightBump, rightTrench;
    // public static double leftBumpDist, leftTrenchDist, rightBumpDist, rightTrenchDist;
    // public static double leftBumpTime, leftTrenchTime, rightBumpTime, rightTrenchTime;

    // public static final double trenchPassSpeed = 3.5; //From pathplanner
    // public static final double bumpPassSpeed = 2.4; //From pathplanner

    // public static final double crossingDist = 2.5; 

    

    /** Updates goalLocation and currentZone based on robot pose */
    public static void updateActiveGoal(Pose2d robotPose) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return;
        boolean allianceColor = alliance.isPresent() && alliance.get() == Alliance.Red;
        if (allianceColor) {
            if (robotPose.getX() < redLine) {
                if (robotPose.getY() < middleY) {
                    currentZone = FieldZone.MAILING_LEFT;
                    goalLocation = new Translation2d(goalLeftRedX, goalLeftRedY);
                    passing = true;
                } else {
                    currentZone = FieldZone.MAILING_RIGHT;
                    goalLocation = new Translation2d(goalRightRedX, goalRightRedY);
                    passing = true;
                }
            } else {
                currentZone = FieldZone.HUB;
                goalLocation = new Translation2d(goalRedX, goalRedY);
                passing = false;
            }
        } else {
            // Blue Alliance Logic
            if (robotPose.getX() > blueLine) {
                if (robotPose.getY() < middleY) {
                    currentZone = FieldZone.MAILING_RIGHT;
                    goalLocation = new Translation2d(goalRightBlueX, goalRightBlueY);
                    passing = true;
                } else {
                    currentZone = FieldZone.MAILING_LEFT;
                    goalLocation = new Translation2d(goalLeftBlueX, goalLeftBlueY);
                    passing = true;
                }
            } else {
                currentZone = FieldZone.HUB;
                goalLocation = new Translation2d(goalBlueX, goalBlueY);
                passing = false;
            }
        }
        // Calculate distance for general use
        distToGoal = goalLocation.minus(robotPose.getTranslation()).getNorm();
    }


    public static void updateMovingTarget(Pose2d currentPose, ChassisSpeeds fieldSpeeds) {
        double refinedDist = currentPose.getTranslation().getDistance(goalLocation);
        double refinedTOF = 0;
        Translation2d tempVTarget = goalLocation;

        for (int i = 0; i < 5; i++) {
            refinedTOF = EquationConstants.calculateTimeOfFlight(refinedDist);
            tempVTarget = new Translation2d(
                goalLocation.getX() - (fieldSpeeds.vxMetersPerSecond * refinedTOF),
                goalLocation.getY() - (fieldSpeeds.vyMetersPerSecond * refinedTOF)
            );
            refinedDist = currentPose.getTranslation().getDistance(tempVTarget);
        }
        
        vGoalDist = refinedDist;
        vTarget = tempVTarget;
    }






    // public void setPoints(CommandSwerveDrivetrain drivetrain) {
    //     var alliance = DriverStation.getAlliance();
    //     isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    //     Pose2d currentPose = drivetrain.getState().Pose;

    //     if (isRedAlliance) { //Red Alliance

    //         if (currentPose.getX() > FieldConstants.redLine) { //In Red Alliance Zone
    //             FieldConstants.leftBump = new Pose2d(13, 2.5, new Rotation2d(0));
    //             FieldConstants.leftTrench = new Pose2d(13, 0.6, new Rotation2d(0));

    //             FieldConstants.rightBump = new Pose2d(13, 5.5, new Rotation2d(0));
    //             FieldConstants.rightTrench = new Pose2d(13, 7.4, new Rotation2d(0)); 
    //             System.out.println("Red Alliance");
    //         } else { //In Neutral Zone
    //             FieldConstants.leftBump = new Pose2d(10.8, 2.5, new Rotation2d(0));
    //             FieldConstants.leftTrench = new Pose2d(10.8, 0.6, new Rotation2d(0));

    //             FieldConstants.rightBump = new Pose2d(10.8, 5.5, new Rotation2d(0));
    //             FieldConstants.rightTrench = new Pose2d(10.8, 7.4, new Rotation2d(0));
    //             System.out.println("Red Neutral");
    //         }


    //     } else { //Blue Alliance

    //         if (currentPose.getX() < FieldConstants.blueLine) { //In Blue Alliance Zone
    //             FieldConstants.leftBump = new Pose2d(3.5, 5.48, new Rotation2d(0));
    //             FieldConstants.leftTrench = new Pose2d(3.5, 7.4, new Rotation2d(0));

    //             FieldConstants.rightBump = new Pose2d(3.5, 2.5, new Rotation2d(0));
    //             FieldConstants.rightTrench = new Pose2d(3.5, 0.65, new Rotation2d(0));
    //             System.out.println("Blue Alliance");
    //         } else { //In Neutral Zone
    //             FieldConstants.leftBump = new Pose2d(6, 5.48, new Rotation2d(0));
    //             FieldConstants.leftTrench = new Pose2d(6, 7.4, new Rotation2d(0));

    //             FieldConstants.rightBump = new Pose2d(6, 2.5, new Rotation2d(0));
    //             FieldConstants.rightTrench = new Pose2d(6, 0.65, new Rotation2d(0));
    //             System.out.println("Blue Neutral");
    //         }
    //     }
    // }


}
