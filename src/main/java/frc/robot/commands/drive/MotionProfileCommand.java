package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;


public class MotionProfileCommand extends CommandBase {
    
    private static final double ramseteB = 2;
    private static final double ramseteZeta = 0.7;

    private final DriveSubsystem driveSubsystem;
    private Trajectory trajectory;
    private final RamseteController controller = new RamseteController(ramseteB, ramseteZeta);
    private final Timer timer = new Timer();

    public MotionProfileCommand(DriveSubsystem drive, String pathName, boolean reverse) {
        addRequirements(drive);
        driveSubsystem = drive;

        double maxVelocity, maxAcceleration;

        maxVelocity = Constants.DriveConstants.maxVelocity;
        maxAcceleration = Constants.DriveConstants.maxAcceleration;

        try {
            trajectory = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration, reverse);
        } catch (TrajectoryParameterizer.TrajectoryGenerationException exception) {
            trajectory = new Trajectory();
            DriverStation.reportError("Failed to load trajectory", false);
        }
    }

    double getTime() {
        return trajectory.getTotalTimeSeconds();
    }

    @Override
    public void initialize() {
        driveSubsystem.setRobotPos(trajectory.getInitialPose());
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Trajectory.State setpoint = trajectory.sample(timer.get());
        ChassisSpeeds speeds = controller.calculate(driveSubsystem.getRobotPos(), setpoint);
        driveSubsystem.voltageDrive(speeds);
    }
}
