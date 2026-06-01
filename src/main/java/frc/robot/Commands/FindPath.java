package frc.robot.Commands;

import java.lang.reflect.Field;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FindPath extends Command{
    CommandSwerveDrivetrain drivetrain;
    boolean isRedAlliance;
    double leftBumpDist, leftTrenchDist, rightBumpDist, rightTrenchDist;

    List<Pose2d> redTargetsAlliance = List.of(
        new Pose2d(13.075, 0.668, new Rotation2d()),
        new Pose2d(13.075, 2.369, new Rotation2d()),
        new Pose2d(13.075, 5.48, new Rotation2d()),
        new Pose2d(13.075, 7.368, new Rotation2d())
    );
    List<Pose2d> redTargetsNeutral = List.of(
        new Pose2d(10.743, 7.395, new Rotation2d()),
        new Pose2d(10.743, 5.537, new Rotation2d()),
        new Pose2d(10.743, 2.507, new Rotation2d()),
        new Pose2d(10.743, 0.608, new Rotation2d())
    );
    List<Pose2d> blueTargetsAlliance = List.of(
        new Pose2d(3.496, 0.679, new Rotation2d()),
        new Pose2d(3.496, 2.512, new Rotation2d()),
        new Pose2d(3.496, 2.512, new Rotation2d()),
        new Pose2d(3.496, 7.373, new Rotation2d())
    );
    List<Pose2d> blueTargetsNeutral = List.of(
        new Pose2d(5.828, 7.456, new Rotation2d()),
        new Pose2d(5.828, 5.531, new Rotation2d()),
        new Pose2d(5.828, 2.520, new Rotation2d()),
        new Pose2d(5.828, 0.665, new Rotation2d())
    );
    

    public FindPath(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        isRedAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        if (isRedAlliance) { //Red Alliance
            if (currentPose.getX() > FieldConstants.redLine) { //In Red Alliance Zone
                //FieldConstants.wantedPose = currentPose.nearest(redTargetsAlliance);
                //leftBumpDist = currentPose.minus()
                System.out.println("Red Alliance");
            } else { //In Neutral Zone
                //FieldConstants.wantedPose = currentPose.nearest(redTargetsNeutral);
                System.out.println("Red Neutral");
            }
        } else { //Blue Alliance
            if (currentPose.getX() < FieldConstants.blueLine) { //In Red Alliance Zone
                //FieldConstants.wantedPose = currentPose.nearest(blueTargetsAlliance);
                System.out.println("Blue Alliance");
            } else { //In Neutral Zone
                //FieldConstants.wantedPose = currentPose.nearest(redTargetsNeutral);
                System.out.println("Blue Neutral");
            }
        }
    }
}
