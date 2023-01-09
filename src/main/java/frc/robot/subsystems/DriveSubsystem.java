// A LOT OF THIS CODE WAS REFERENCED FROM 7034'S 2022 CODE
// https://github.com/2BDetermined-7034/2022-Rapid-React

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;


public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax frontLeft, frontRight, backLeft, backRight;
    private final RelativeEncoder leftEnc, rightEnc;

    private final AHRS gyro;

    private final DifferentialDrive drive;

    private final DifferentialDriveOdometry odometry;
    DifferentialDriveKinematics kinematics;

    private double leftSpeed;
    private double rightSpeed;

    private double leftOffset;
    private double rightOffset;

    public DriveSubsystem() {
        // Creates the motors
        frontLeft = new CANSparkMax(Constants.DriveConstants.kFrontLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight = new CANSparkMax(Constants.DriveConstants.kFrontRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        backLeft = new CANSparkMax(Constants.DriveConstants.kBackLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        backRight = new CANSparkMax(Constants.DriveConstants.kBackRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

        frontLeft.setInverted(false);
        frontRight.setInverted(true);
        backLeft.setInverted(false);
        backRight.setInverted(true);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        drive = new DifferentialDrive(frontLeft, frontRight);
        kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth);

        frontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

        frontLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftEnc = frontLeft.getEncoder();
        rightEnc = frontRight.getEncoder();

        // Creates the gyro
        gyro = new AHRS(SPI.Port.kMXP);

        odometry = new DifferentialDriveOdometry(getCurrentAngle(), leftOffset, rightOffset);
        resetEncoders();
    }

    /**
     * Gets the current position of the robot
     * @return the current position of the robot
     */
    public Pose2d getRobotPos() {
        return odometry.getPoseMeters();
    }

    /** 
     * Gets the current robot velocity
     * @return the current robot velocity
     */
    public DifferentialDriveWheelSpeeds getWheelVelocity() {
        return new DifferentialDriveWheelSpeeds(leftEnc.getVelocity(), rightEnc.getVelocity());
    }

    public double getRightEncoderPosition() {
        return rightEnc.getPosition();
    }

    public double getLeftEncoderPosition() {
        return leftEnc.getPosition();
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetEncoders() {
        leftOffset = leftEnc.getPosition();
        rightOffset = rightEnc.getPosition();
    }

    public void setRobotPos(Pose2d startingPose){
        resetEncoders();
        odometry.resetPosition(getCurrentAngle(), leftOffset, rightOffset, startingPose);
    }
    
    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
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

    public void voltageDrive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        frontLeft.setVoltage(wheelSpeeds.leftMetersPerSecond);
        frontRight.setVoltage(wheelSpeeds.rightMetersPerSecond);
        drive.feed();
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
