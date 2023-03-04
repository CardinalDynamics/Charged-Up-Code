package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;

public class TankDrive {
    private CANSparkMax[] motors;
    private DifferentialDrive drive;
    private Pose2d pose;

    public TankDrive() {
        this.motors = new CANSparkMax[4];
        this.motors[0] = new CANSparkMax(Constants.frontLeftMotorPort, MotorType.kBrushless);
        this.motors[1] = new CANSparkMax(Constants.backLeftMotorPort, MotorType.kBrushless);
        this.motors[2] = new CANSparkMax(Constants.frontRightMotorPort, MotorType.kBrushless);
        this.motors[3] = new CANSparkMax(Constants.backRightMotorPort, MotorType.kBrushless);

        this.motors[0].restoreFactoryDefaults();
        this.motors[1].restoreFactoryDefaults();
        this.motors[2].restoreFactoryDefaults();
        this.motors[3].restoreFactoryDefaults();

        this.motors[0].setInverted(true);
        this.motors[1].setInverted(true);
        this.motors[2].setInverted(false);
        this.motors[3].setInverted(false);

        this.motors[0].setIdleMode(IdleMode.kCoast);
        this.motors[1].setIdleMode(IdleMode.kCoast);
        this.motors[2].setIdleMode(IdleMode.kCoast);
        this.motors[3].setIdleMode(IdleMode.kCoast);

        this.motors[1].follow(this.motors[0]);
        this.motors[3].follow(this.motors[2]);

        SmartDashboard.putNumber("Drive Current Limit", Constants.driveCurrentLimit);

        this.drive = new DifferentialDrive(this.motors[0], this.motors[2]);

        this.pose = new Pose2d();
    }

    public void updateSpeedTank(double leftSpeed, double rightSpeed) {
        int limit = (int) Math.round(SmartDashboard.getNumber("Drive Current Limit", Constants.driveCurrentLimit));
        motors[0].setSmartCurrentLimit(limit);
        motors[1].setSmartCurrentLimit(limit);
        motors[2].setSmartCurrentLimit(limit);
        motors[3].setSmartCurrentLimit(limit);

        this.drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void updateSpeedArcade(double speed, double turn) {
        int limit = (int) Math.round(SmartDashboard.getNumber("Drive Current Limit", Constants.driveCurrentLimit));
        motors[0].setSmartCurrentLimit(limit);
        motors[1].setSmartCurrentLimit(limit);
        motors[2].setSmartCurrentLimit(limit);
        motors[3].setSmartCurrentLimit(limit);

        this.drive.arcadeDrive(speed, turn);
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetPose() {
        pose = new Pose2d();
    }

    public void setBrakeMode(boolean brake) {
        if (brake) {
            this.motors[0].setIdleMode(IdleMode.kBrake);
            this.motors[1].setIdleMode(IdleMode.kBrake);
            this.motors[2].setIdleMode(IdleMode.kBrake);
            this.motors[3].setIdleMode(IdleMode.kBrake);
        } else {
            this.motors[0].setIdleMode(IdleMode.kCoast);
            this.motors[1].setIdleMode(IdleMode.kCoast);
            this.motors[2].setIdleMode(IdleMode.kCoast);
            this.motors[3].setIdleMode(IdleMode.kCoast);
        }
    }

}
