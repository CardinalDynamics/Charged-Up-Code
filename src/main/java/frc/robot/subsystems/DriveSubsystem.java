package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
    private final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
    private final CANSparkMax backLeftMotor = new CANSparkMax(Constants.DriveConstants.kBackLeftMotorPort, MotorType.kBrushless);
    private final CANSparkMax backRightMotor = new CANSparkMax(Constants.DriveConstants.kBackRightMotorPort, MotorType.kBrushless);

    private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);

    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    public DriveSubsystem() {
        super();

        rightMotors.setInverted(true);

        addChild("Drive", drive);
        addChild("Gyro", gyro);

        gyro.calibrate();

        frontLeftMotor.setSmartCurrentLimit(40);
        frontRightMotor.setSmartCurrentLimit(40);
        backLeftMotor.setSmartCurrentLimit(40);
        backRightMotor.setSmartCurrentLimit(40);
    }

    /**
     * Logs the gyro angle to the SmartDashboard
     */
    public void log() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    }

    /**
     * Drives the robot using tank drive
     * @param leftSpeed
     * @param rightSpeed
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        // This is the method that will be used to drive the robot
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Drives the robot using arcade drive
     * @param speed
     * @param rotation
     */
    public void arcadeDrive(double speed, double rotation) {
        // This is the method that will be used to drive the robot
        drive.arcadeDrive(speed, rotation);
    }

    public void resetGyro() {
        // Resets the gyro to 0 degrees
        gyro.reset();
    }

    public double getGyroAngle() {
        // Returns the angle of the robot in degrees
        return gyro.getAngle();
    }

    public void stop() {
        // This is a safety measure to stop the robot if the driver loses control
        drive.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        log();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
