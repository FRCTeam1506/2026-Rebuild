package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.EquationConstants;

public class FieldConstants {
    // Current State
    public static Translation2d goalLocation = new Translation2d(0,0);
    public static double distToGoal;
    
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
                } else {
                    currentZone = FieldZone.MAILING_RIGHT;
                    goalLocation = new Translation2d(goalRightRedX, goalRightRedY);
                }
            } else {
                currentZone = FieldZone.HUB;
                goalLocation = new Translation2d(goalRedX, goalRedY);
            }
        } else {
            // Blue Alliance Logic
            if (robotPose.getX() > blueLine) {
                if (robotPose.getY() < middleY) {
                    currentZone = FieldZone.MAILING_RIGHT; // Relative to Blue wall
                    goalLocation = new Translation2d(goalRightBlueX, goalRightBlueY);
                } else {
                    currentZone = FieldZone.MAILING_LEFT;
                    goalLocation = new Translation2d(goalLeftBlueX, goalLeftBlueY);
                }
            } else {
                currentZone = FieldZone.HUB;
                goalLocation = new Translation2d(goalBlueX, goalBlueY);
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


}
