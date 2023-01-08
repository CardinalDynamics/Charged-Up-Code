package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;



// I'll be honest, I have no clue what any of this does, but it works.

public class ComplexAuto extends SequentialCommandGroup {
    
    private final DriveSubsystem drivetrain;
    private final Trajectory trajectory;
    private Timer timer = new Timer();

    public SwerveControllerFollower(DriveSubsystem drivetrain, Trajectory trajectory) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
        addRequirements(drivetrain);
    }
    
    
    /**
     * ComplexAuto is a command group that drives backwards for 2.5 seconds, and then shoots a ball.
     * 
     * @param driveSubsystem The drive subsystem
     *
     */
    public ComplexAuto(DriveSubsystem driveSubsystem) {
        addCommands(
            
            // Thank you to Zack from 1540 for actually knowing how to write this.
            // a command that starts and then ends i guess
            new StartEndCommand(
                () -> driveSubsystem.tankDrive(0.25, 0.25),
                () -> driveSubsystem.tankDrive(0, 0),
                driveSubsystem
            ).withTimeout(2.5));
        
    }
}