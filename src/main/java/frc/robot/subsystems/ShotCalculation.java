package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShotCalculation extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;


    //Positions
    //Variables for getting angle to goal.
    double goalRedX = 12; //Red Goal
    double goalRedY = 4.034536;
    double goalBlueX = -12; //Blue Goal
    double goalBlueY = 4.034536;
    Translation2d goalLocation = new Translation2d(0, 0);

    //FIELD LOCATIONS:
    final double redLine = 11.5; //used to be 12.6, made it 11.5 for more accurate zone of when we want to do mailing funciton
    final double middleY = 4;
    final double blueLine = 4.1;

    final double goalRightRedY = 7;//7.211
    final double goalRightRedX = 15;

    final double goalLeftRedY = 1.5; //0.8
    final double goalLeftRedX = 15;

    final double goalRightBlueY = 1.2; //7.756
    final double goalRightBlueX = 2;

    final double goalLeftBlueY = 7; //3
    final double goalLeftBlueX = 1.5;


    //Math variables
    public static double theta;
    double thetaX, thetaY;
    double angleToGoal;
    double toDegree;
    double angleToPos;
    public static double wantedHeading;
    double dist;
    Translation2d targetVec;
    double distToGoal;

    //Robot Poses
    Pose2d robotPose;
    double robotPoseX;
    double robotPoseY;
    double omega;
    double heading;

    Translation2d rotationalVelocityField;

    //Virtual Robot Poses
    double vRobotY;
    double vRobotX;
    double vRotationalRobotY;
    double vRotationalRobotX;
    double totalFieldVy;
    double totalFieldVx;
    Translation2d vRobotPose;
    

    //Extra
    double flightTimeMultiplier = 1.3; //If turret doesn't offset enough. Tune this if needed


    public ShotCalculation(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    


    @Override
    public void periodic() {
        omega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
        robotPose = drivetrain.getState().Pose;
        robotPoseX = drivetrain.getState().Pose.getX();
        robotPoseY = drivetrain.getState().Pose.getY();

        vRotationalRobotY = omega * robotPoseX;
        vRotationalRobotX = -omega * robotPoseY;
        rotationalVelocityField = new Translation2d(vRotationalRobotX, vRotationalRobotY).rotateBy(drivetrain.getState().Pose.getRotation());

        heading = drivetrain.getState().Pose.getRotation().getDegrees();
        theta = Math.atan2(thetaY, thetaX);
        toDegree = Math.toDegrees(theta);
        wantedHeading = toDegree - heading;

        targetVec = goalLocation.minus(vRobotPose);
        dist = targetVec.getNorm();
        distToGoal = dist;
    }
}
