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

    public double distanceAssist() {
        double distanceError = estimateDistance() - Constants.targetDistance;
        if (!getHasTargets() || Math.abs(distanceError) < 0.5) {
            SmartDashboard.putNumber("Distance Adjustment", 0);
            return 0.0;
        }
        double adjustment = distanceError * 0.225;
        adjustment = Math.min(Constants.maxTurnSpeed, Math.max(-Constants.maxTurnSpeed, adjustment));
        SmartDashboard.putNumber("Distance Adjustment", adjustment);
        return adjustment;
    }

    public double steeringAssist() {
        double offset = getXAngleOffset() - Math.atan(12 / estimateDistance());
        if (!getHasTargets() || Math.abs(offset) < Constants.minTurnAngle) {
            SmartDashboard.putNumber("Turning Adjustment", 0);
            return 0;
        }
        double adjustment = pidController.calculate(offset);
        adjustment = Math.min(Constants.maxTurnSpeed, Math.max(-Constants.maxTurnSpeed, adjustment));
        adjustment = pidController.atSetpoint() ? 0 : -adjustment;
        SmartDashboard.putNumber("Turning Adjustment", adjustment);
        return adjustment;
    }
    
}
