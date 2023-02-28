package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

public class Autos {
    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private String autoSelected;
    private final RamseteController ramseteController = new RamseteController();
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
    PathPlannerTrajectory pathPlannerTrajectory;
    PathConstraints pathConstraints;
    RamseteAutoBuilder autoBuilder;

    TankDrive drive = new TankDrive();


    public Autos() {
        autoChooser.setDefaultOption("Red 1", "Red 1");
        autoChooser.addOption("Red 2", "Red 2");
        autoChooser.addOption("Red 3", "Red 3");
        autoChooser.addOption("Blue 1", "Blue 1");
        autoChooser.addOption("Blue 2", "Blue 2");
        autoChooser.addOption("Blue 3", "Blue 3");
        SmartDashboard.putData("Auto choices", autoChooser);

        pathConstraints = new PathConstraints(Constants.maxVelocity, Constants.maxAcceleration);

        autoBuilder = new RamseteAutoBuilder(
                drive::getPose,
                drive::resetPose,
                kinematics,
                new PIDConstants(0, 0, 0),
                new PIDConstants(0, 0, 0),
                ramseteController,

            );

    }

    public void autoInit() {
        autoSelected = autoChooser.getSelected();
    }

    public void autoPeriodic() {
        switch (autoSelected) {
            case "Red 1":
                blue3();
                break;
            case "Red 2":
                blue2();
                break;
            case "Red 3":
                blue1();
                break;
            case "Blue 1":
                blue1();
                break;
            case "Blue 2":
                blue2();
                break;
            case "Blue 3":
                blue3();
                break;
            default:
                break;
        }
    }

    private void blue1() {
        pathPlannerTrajectory = PathPlanner.loadPath("Blue 1", pathConstraints);
    }

    private void blue2() {
        pathPlannerTrajectory = PathPlanner.loadPath("Blue 2", pathConstraints);
    }

    private void blue3() {
        pathPlannerTrajectory = PathPlanner.loadPath("Blue 3", pathConstraints);
    }
}
