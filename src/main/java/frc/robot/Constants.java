package frc.robot;

public class Constants {
    // Drivetrain
    public static final int frontLeftMotorPort = 1;
    public static final int backLeftMotorPort = 2;
    public static final int frontRightMotorPort = 3;
    public static final int backRightMotorPort = 4;
    public static final int driveCurrentLimit = 80;

    // Arm
    public static final int armMotorPort = 5;
    public static final int armSolenoidPortA = 10;
    public static final int armSolenoidPortB = 11;
    public static final int armSolenoidPortC = 12;
    public static final int armSolenoidPortD = 13;

    // Manipulator
    public static final int manipulatorSolenoidPortA = 14;
    public static final int manipulatorSolenoidPortB = 15;

    // Controllers
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
    public static final double deadband = 0.1;

    // Vision
    public static final double visionCameraAngle = 0.0;
    public static final double visionCameraHeight = 15.13 - 26.5; // target - camera
    
}
