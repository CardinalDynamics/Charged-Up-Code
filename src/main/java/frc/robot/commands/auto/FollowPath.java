package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPath {
    
    private final RamseteCommand ramseteCommand;
    private final DriveSubsystem drive;
    private final Trajectory path;
    public FollowPath(DriveSubsystem drive, String path, boolean reversed) {
        this.drive = drive;
        Trajectory trajectory;
        try {
            trajectory = PathPlanner.loadPath(path, Constants.DriveConstants.maxVelocity, Constants.DriveConstants.maxAcceleration, reversed);
        } catch (TrajectoryParameterizer.TrajectoryGenerationException exception) {
            trajectory = new Trajectory();
            DriverStation.reportError("Failed to load trajectory", false);
        }
        this.path = trajectory;
        ramseteCommand = new RamseteCommand(
            trajectory,
            drive::getRobotPos,
            new RamseteController(Constants.DriveConstants.b, Constants.DriveConstants.zeta),
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            drive.getKinematics(),
            drive::getWheelVelocity,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            drive::tankDriveVolts,
            drive);
    }
    
    public RamseteCommand getRamseteCommand() {
        return ramseteCommand;
    }
}