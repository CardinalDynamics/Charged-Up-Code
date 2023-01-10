package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class SetOdometry extends CommandBase {
    private final DriveSubsystem drive;
    private final Trajectory path;

    public SetOdometry(DriveSubsystem drive, String path) {
        this.drive = drive;
        Trajectory trajectory;
        try {
            trajectory = PathPlanner.loadPath(path, Constants.DriveConstants.maxVelocity, Constants.DriveConstants.maxAcceleration, false);
        } catch (TrajectoryParameterizer.TrajectoryGenerationException exception) {
            trajectory = new Trajectory();
            DriverStation.reportError("Failed to load trajectory", false);
        }
        this.path = trajectory;
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        drive.setRobotPos(path.getInitialPose());
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean inturrupted) {
        
    }
}
