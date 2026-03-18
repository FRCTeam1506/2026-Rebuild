package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.alignVariables;

public class ShotCalculation extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;

    public static int shootMode = 1;
    boolean red;
    Optional<Alliance> alliance = DriverStation.getAlliance();

    public InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();

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
    double preHeading;
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

        if (alliance.get() == Alliance.Red) {
            red = true;
        } else {
            red = false;
        }


        timeOfFlight.put(1.0, 1.0);
    }

    public void shootModeUp() {
        shootMode += 1;
    }
    public void shootModeDown() {
        shootMode -= 1;
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

        totalFieldVy = drivetrain.getState().Speeds.vyMetersPerSecond + rotationalVelocityField.getY();
        totalFieldVx = drivetrain.getState().Speeds.vxMetersPerSecond + rotationalVelocityField.getX();
        
        //Times time of flight
        vRobotY = robotPose.getY() + (totalFieldVy * timeOfFlight.get(alignVariables.distToGoal) * flightTimeMultiplier);
        vRobotX = robotPose.getX() + (totalFieldVx * timeOfFlight.get(alignVariables.distToGoal) * flightTimeMultiplier);
        vRobotPose = new Translation2d(vRobotX, vRobotY);

        switch (shootMode) {
            case 0: //Keep turret at zero
              break;
            case 1: //Change goal based on robot position
                if(red == true) {
                    if (robotPoseX > redLine) {
                        goalLocation = new Translation2d(goalRedX, goalRedY);
                        thetaX = goalLocation.getX() - vRobotX;
                        thetaY = goalLocation.getY() - vRobotY;
                    }
                    if(robotPoseY < 4 && robotPoseX < redLine) {//left (from red perspective)
                        goalLocation = new Translation2d(goalLeftRedX, goalLeftRedY);
                        thetaX = goalLocation.getX() - vRobotX;
                        thetaY = goalLocation.getY() - vRobotY;
                    }
                    if(robotPoseY > 4 && robotPoseX < redLine) { 
                        goalLocation = new Translation2d(goalRightRedX, goalRightRedY);
                        thetaX = goalLocation.getX() - vRobotX;
                        thetaY = goalLocation.getY() - vRobotY;
                    }
                }
                else { //blue
                    if (robotPoseX < blueLine) {
                        goalLocation = new Translation2d(goalBlueX, goalBlueY);
                        thetaX = goalLocation.getX() - vRobotX;
                        thetaY = goalLocation.getY() - vRobotY;
                    }
                    if(robotPoseY < 4 && robotPoseX > blueLine) { //left (from red perspective)
                        goalLocation = new Translation2d(goalLeftBlueX, goalLeftBlueY);
                        thetaX = goalLocation.getX() - vRobotX;
                        thetaY = goalLocation.getY() - vRobotY;
                    }
                    if(robotPoseY > 4 && robotPoseX > blueLine) {
                        goalLocation = new Translation2d(goalRightBlueX, goalRightBlueY);
                        thetaX = goalLocation.getX() - vRobotX;
                        thetaY = goalLocation.getY() - vRobotY;
                    }
                    
                }
                //Constants.manualTurret = false;
                break;
            case 2: //Goal only
                if (red == true) {
                    goalLocation = new Translation2d(goalRedX, goalRedY);
                    thetaX = goalLocation.getX() - vRobotX;
                    thetaY = goalLocation.getY() - vRobotY;
                } else {
                    goalLocation = new Translation2d(goalBlueX, goalBlueY);
                    thetaX = goalLocation.getX() - vRobotX;
                    thetaY = goalLocation.getY() - vRobotY;
                }
                //Constants.manualTurret = false;
                break;
            case 3: //Aim to the left side for mailing DRIVER PERSPECTIVE
                if (red == true) {
                goalLocation = new Translation2d(goalLeftRedX, goalLeftRedY);
                thetaX = goalLocation.getX() - vRobotX;
                thetaY = goalLocation.getY() - vRobotY;
                } else {
                goalLocation = new Translation2d(goalLeftBlueX, goalLeftBlueY);
                thetaX = goalLocation.getX() - vRobotX;
                thetaY = goalLocation.getY() - vRobotY;
                }
                //Constants.manualTurret = false;
                break;
            case 4: //Aim to the right side for mailing DRIVER PERSPECTIVE
                if (red == true) {
                goalLocation = new Translation2d(goalRightRedX, goalRightRedY);
                thetaX = goalLocation.getX() - vRobotX;
                thetaY = goalLocation.getY() - vRobotY;
                } else {
                goalLocation = new Translation2d(goalRightBlueX, goalRightBlueY);
                thetaX = goalLocation.getX() - vRobotX;
                thetaY = goalLocation.getY() - vRobotY;
                }
                //Constants.manualTurret = false;
                break;
        
            default: //Anything else stop turret
                //Constants.manualTurret = true;
                break;
        }

        heading = drivetrain.getState().Pose.getRotation().getDegrees();
        theta = Math.toDegrees(Math.atan2(thetaY, thetaX)); //degree value of theta
        //toDegree = Math.toDegrees(theta);
        //preHeading = toDegree - heading; 
        //wantedHeading = MathUtil.inputModulus(preHeading, -180, 180);

        targetVec = goalLocation.minus(vRobotPose);
        dist = targetVec.getNorm();
        alignVariables.distToGoal = dist;

        if (alliance.get() == Alliance.Red) {
            red = true;
        } else {
            red = false;
        }
    }
}
