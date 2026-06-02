package frc.robot.Commands.AutoPathing;

import java.lang.reflect.Field;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand; 

public class Pathing {
    CommandSwerveDrivetrain drivetrain;
    boolean isRedAlliance;
    
    double shortestTime;

    public static int crossingZone; //1 = Left Trench, 2 = Left Bump, 3 = Right Bump, 4 = Right Trench
    public static Pose2d leftBump, leftTrench, rightBump, rightTrench;
    public static double leftBumpDist, leftTrenchDist, rightBumpDist, rightTrenchDist;
    public static double leftBumpTime, leftTrenchTime, rightBumpTime, rightTrenchTime;

    public static final double trenchPassSpeed = 3.5; //From pathplanner
    public static final double bumpPassSpeed = 2.4; //From pathplanner

    public static final double crossingDist = 2.5; 

    double bumpTime = crossingDist / bumpPassSpeed;
    double trenchTime = crossingDist / trenchPassSpeed;

    public static Pose2d wantedPose;
    public static Pose2d crossingPath;

    PathConstraints constraints;

    public Pathing(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    } 

    public void setPoints() {
        var alliance = DriverStation.getAlliance();
        isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        Pose2d currentPose = drivetrain.getState().Pose;

        if (isRedAlliance) { //Red Alliance

            if (currentPose.getX() > FieldConstants.redLine) { //In Red Alliance Zone
                leftBump = new Pose2d(13, 2.5, new Rotation2d(0));
                leftTrench = new Pose2d(13, 0.6, new Rotation2d(0));

                rightBump = new Pose2d(13, 5.5, new Rotation2d(0));
                rightTrench = new Pose2d(13, 7.4, new Rotation2d(0)); 
                System.out.println("Red Alliance");
            } else { //In Neutral Zone
                leftBump = new Pose2d(10.8, 2.5, new Rotation2d(0));
                leftTrench = new Pose2d(10.8, 0.6, new Rotation2d(0));

                rightBump = new Pose2d(10.8, 5.5, new Rotation2d(0));
                rightTrench = new Pose2d(10.8, 7.4, new Rotation2d(0));
                System.out.println("Red Neutral");
            }


        } else { //Blue Alliance

            if (currentPose.getX() < FieldConstants.blueLine) { //In Blue Alliance Zone
                leftBump = new Pose2d(3.5, 5.48, new Rotation2d(0));
                leftTrench = new Pose2d(3.5, 7.4, new Rotation2d(0));

                rightBump = new Pose2d(3.5, 2.5, new Rotation2d(0));
                rightTrench = new Pose2d(3.5, 0.65, new Rotation2d(0));
                System.out.println("Blue Alliance");
            } else { //In Neutral Zone
                leftBump = new Pose2d(6, 5.48, new Rotation2d(0));
                leftTrench = new Pose2d(6, 7.4, new Rotation2d(0));

                rightBump = new Pose2d(6, 2.5, new Rotation2d(0));
                rightTrench = new Pose2d(6, 0.65, new Rotation2d(0));
                System.out.println("Blue Neutral");
            }
        }
    }
    
    public Pose2d shortestPose(CommandSwerveDrivetrain mDrivetrain) {
        setPoints();
        
        leftBumpDist = mDrivetrain.getState().Pose.getTranslation().getDistance(leftBump.getTranslation());
        leftTrenchDist = mDrivetrain.getState().Pose.getTranslation().getDistance(leftTrench.getTranslation());
        rightBumpDist = mDrivetrain.getState().Pose.getTranslation().getDistance(rightBump.getTranslation());
        rightTrenchDist = mDrivetrain.getState().Pose.getTranslation().getDistance(rightTrench.getTranslation());

        leftBumpTime = (leftBumpDist / trenchPassSpeed) + bumpTime; //use trench pass speed as max speed
        leftTrenchTime = (leftTrenchDist / trenchPassSpeed) + trenchTime;
        rightBumpTime = (rightBumpDist / trenchPassSpeed) + bumpTime;
        rightTrenchTime = (rightTrenchDist / trenchPassSpeed) + trenchTime;
        
        shortestTime = leftBumpTime;
        wantedPose = leftBump;

        if (leftTrenchTime < shortestTime) {
            shortestTime = leftTrenchTime;
            wantedPose = leftTrench;
        }

        if (rightBumpTime < shortestTime) {
            shortestTime = rightBumpTime;
            wantedPose = rightBump;
        }

        if (rightTrenchTime < shortestTime) {
            shortestTime = rightTrenchTime;
            wantedPose = rightTrench;
        }

        return wantedPose;
    }


    public Pose2d crossingPath(CommandSwerveDrivetrain mDrivetrain, Pose2d toCrossPose) {
        var alliance = DriverStation.getAlliance();
        isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        if (isRedAlliance) {
            if (mDrivetrain.getState().Pose.getX() > FieldConstants.redLine) {
                crossingPath = new Pose2d(toCrossPose.getX() - 2.5, toCrossPose.getY(), new Rotation2d());
            } else {
                crossingPath = new Pose2d(toCrossPose.getX() + 2.5, toCrossPose.getY(), new Rotation2d());
            }
        } else {
            if (mDrivetrain.getState().Pose.getX() < FieldConstants.blueLine) {
                crossingPath = new Pose2d(toCrossPose.getX() + 2.5, toCrossPose.getY(), new Rotation2d());
            } else {
                crossingPath = new Pose2d(toCrossPose.getX() - 2.5, toCrossPose.getY(), new Rotation2d());
            }
        }

        return crossingPath;
    }


    // public Command getPathfinding (Pose2d pose) {
    //     return AutoBuilder.pathfindToPose(pose, constraints, 0.0);
    // }
}
