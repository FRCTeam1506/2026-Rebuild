package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.GoalEndState;

public class DriveToTrench {
    //open field constraints
    private static final PathConstraints OPEN_FIELD_CONSTRAINTS = new PathConstraints(
        DriveConstants.TRENCH_MAX_VEL,
        DriveConstants.TRENCH_MAX_ACCEL,
        DriveConstants.TRENCH_MAX_ANG_VEL,
        DriveConstants.TRENCH_MAX_ANG_ACCEL
    );

    //fixed constraints
    private static final PathConstraints TRENCH_CORRIDOR_CONSTRAINTS = new PathConstraints(
        DriveConstants.TRENCH_MAX_VEL * 0.5,   //slower under trench
        DriveConstants.TRENCH_MAX_ACCEL * 0.75, //slower under trench
        DriveConstants.TRENCH_MAX_ANG_VEL,
        DriveConstants.TRENCH_MAX_ANG_ACCEL
    );

    //the speed for ending path 1 (dynamic) and start of path 2 (fixed)
    private static final double COAST_HANDOFF_VELOCITY = 1.5; 

    /**
     * Automatically calculates and executes a safe two-phase macro through the nearest trench.
     * Phase 1 pathfinds quickly to a safe lineup staging point using open-field constraints.
     * Phase 2 executes a slower, highly controlled straight path through the trench structure.
     * 
     * @param drivetrain Your swerve drive subsystem dependency.
     */
    public static Command create(CommandSwerveDrivetrain drivetrain) {
        return new DeferredCommand(() -> {
            Pose2d currentPose = drivetrain.getState().Pose;
            double entryX;
            double exitX;
            double trenchY;
            Rotation2d targetHeading;
            double lineupOffsetDirection;

            var alliance = DriverStation.getAlliance();
            boolean isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            boolean chooseLeftZone = currentPose.getY() < FieldConstants.middleY;

            if (isRedAlliance) {
                entryX = FieldConstants.RED_TRENCH_X_ENTRY;
                exitX = FieldConstants.RED_TRENCH_X_EXIT;
                trenchY = chooseLeftZone ? FieldConstants.RED_LEFT_TRENCH_Y : FieldConstants.RED_RIGHT_TRENCH_Y;
                targetHeading = Rotation2d.fromDegrees(0);
                lineupOffsetDirection = -1.0; 
            } else {
                entryX = FieldConstants.BLUE_TRENCH_X_ENTRY;
                exitX = FieldConstants.BLUE_TRENCH_X_EXIT;
                trenchY = chooseLeftZone ? FieldConstants.BLUE_LEFT_TRENCH_Y : FieldConstants.BLUE_RIGHT_TRENCH_Y;
                targetHeading = Rotation2d.fromDegrees(180);
                lineupOffsetDirection = 1.0; 
            }

            Translation2d entryGoal = new Translation2d(entryX, trenchY);
            Translation2d exitGoal = new Translation2d(exitX, trenchY);
            
            Pose2d approachLineup = new Pose2d(
                entryX + (FieldConstants.LINEUP_OFFSET_METERS * lineupOffsetDirection), 
                trenchY,
                targetHeading
            );

            //dynamic pathing when not under the trench
            Command phase1Pathfind = AutoBuilder.pathfindToPose(
                approachLineup, 
                OPEN_FIELD_CONSTRAINTS, //when not going under the trench
                COAST_HANDOFF_VELOCITY 
            );

            //Fixed waypoints rather than dynamic pathing
            List<Pose2d> straightWaypoints = List.of(
                approachLineup,
                new Pose2d(entryGoal, targetHeading),
                new Pose2d(exitGoal, targetHeading)
            );
            
            List<Waypoint> pathplannerWaypoints = PathPlannerPath.waypointsFromPoses(straightWaypoints);
            IdealStartingState startingState = new IdealStartingState(COAST_HANDOFF_VELOCITY, targetHeading);

            PathPlannerPath trenchStraightaway = new PathPlannerPath(
                pathplannerWaypoints,
                TRENCH_CORRIDOR_CONSTRAINTS, //goes slower when going under the trench
                startingState, 
                new GoalEndState(0.0, targetHeading) 
            );
            
            trenchStraightaway.preventFlipping = true; 
            Command phase2StraightDrive = AutoBuilder.followPath(trenchStraightaway);

            return Commands.sequence(
                phase1Pathfind,
                phase2StraightDrive
            );

        }, Set.of(drivetrain));
    }
}
