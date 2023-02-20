package frc.robot;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive {
    private boolean inverted;
    private CANSparkMax[] motors;

    public Drive() {
        this.motors = new CANSparkMax[4];
        this.motors[0] = new CANSparkMax(Constants.frontLeftMotorPort, MotorType.kBrushless);
        this.motors[1] = new CANSparkMax(Constants.backLeftMotorPort, MotorType.kBrushless);
        this.motors[2] = new CANSparkMax(Constants.frontRightMotorPort, MotorType.kBrushless);
        this.motors[3] = new CANSparkMax(Constants.backRightMotorPort, MotorType.kBrushless);

        this.motors[0].restoreFactoryDefaults();
        this.motors[1].restoreFactoryDefaults();
        this.motors[2].restoreFactoryDefaults();
        this.motors[3].restoreFactoryDefaults();
        
        this.motors[0].setInverted(false);
        this.motors[1].setInverted(false);
        this.motors[2].setInverted(true);
        this.motors[3].setInverted(true);

        this.motors[0].setIdleMode(IdleMode.kCoast);
        this.motors[1].setIdleMode(IdleMode.kCoast);
        this.motors[2].setIdleMode(IdleMode.kCoast);
        this.motors[3].setIdleMode(IdleMode.kCoast);

        this.motors[0].follow(this.motors[1]);
        this.motors[2].follow(this.motors[3]);

        inverted = false;
        SmartDashboard.putNumber("Drive Current Limit", Constants.driveCurrentLimit);
    }

    public void invertDrive() {
        inverted =! inverted;
    }

    public void updateSpeed(double drive, double turn) {
        updateSpeedInternal(drive, turn, true);
    }

    public void updateAutoSpeed(double drive, double turn) {
        updateSpeedInternal(drive, turn, false);
    }

    private void updateSpeedInternal(double drive, double turn, boolean useInverted) {
        int limit = (int) Math.round(SmartDashboard.getNumber("Drive Current Limit", Constants.driveCurrentLimit));
        motors[0].setSmartCurrentLimit(limit);
        motors[1].setSmartCurrentLimit(limit);
        motors[2].setSmartCurrentLimit(limit);
        motors[3].setSmartCurrentLimit(limit);
        
        double[] speeds = new double[4];
        if (useInverted && inverted) {
            speeds[0] = 0 - drive - turn;
            speeds[1] = 0 - drive - turn;
            speeds[2] = 0 - drive + turn;
            speeds[3] = 0 - drive + turn;

        } else {
            speeds[0] = 0 + drive - turn;
            speeds[1] = 0 + drive - turn;
            speeds[2] = 0 + drive + turn;
            speeds[3] = 0 + drive + turn;
        }

        for (int i = 0; i < 4; ++i) {
            this.motors[i].set(speeds[i]);
        }
    }
}
