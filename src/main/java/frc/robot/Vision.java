package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    public NetworkTable table;
    public PIDController pidController;

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        pidController = new PIDController(0.1, 0, 0);
        pidController.setSetpoint(0);
        pidController.setTolerance(0.5);
    }

    public double getXAngleOffset() {
        double tx = table.getEntry("tx").getDouble(0);
        return Double.isNaN(tx) ? 0.0 : tx;
    }
    
    public double getYAngleOffset() {
        double ty = table.getEntry("ty").getDouble(0);
        return Double.isNaN(ty) ? 0.0 : ty;
    }

    public boolean getHasTargets() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public double estimateDistance() {
        double degrees = Constants.visionCameraAngle + getYAngleOffset();
        double distance = Constants.visionCameraHeight / Math.tan(degrees * Math.PI / 180.0);
        SmartDashboard.putNumber("Estimated Distance", distance);
        return distance;
    }
    
}
