package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.limelight.CamMode;

public class Limelight extends SubsystemBase {
    
    private final NetworkTable m_table;

    public Limelight() {
        m_table = NetworkTableInstance.getDefault().getTable("limelight");
        setLights(false);
    }

    public double getXAngle() { return m_table.getEntry("tx").getDouble(0); }
    public double getYAngle() { return m_table.getEntry("ty").getDouble(0); }
    public double getArea() { return m_table.getEntry("ta").getDouble(0); }
    public double getShortSide() { return m_table.getEntry("tshort").getDouble(0); }
    public double getLongSide() { return m_table.getEntry("tlong").getDouble(0); }
    public boolean getDetected() { return m_table.getEntry("tv").getDouble(0) > 0; }
    public double getLastDetected() { return m_table.getEntry("tv").getLastChange(); }

    public CamMode getMode() {
        return CamMode.getFromNetworkTableValue((int) m_table.getEntry("camMode").getDouble(0));
    }

    public void setMode(CamMode mode) {
        m_table.getEntry("camMode").setNumber(mode.getNetworkTableValue());
    }

    public void setLights(boolean on) {
        m_table.getEntry("ledMode").setNumber(on ? 3 : 1);
    }

    public double getEstimatedDistance() {
        return 3.494*Math.pow(Math.tan(Math.toRadians(Constants.vision.Vis_LLAngle+getYAngle())));
    }
}
