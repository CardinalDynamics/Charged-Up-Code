package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Constants {
    // Drivetrain
    public static final int frontLeftMotorPort = 1;
    public static final int backLeftMotorPort = 2;
    public static final int frontRightMotorPort = 3;
    public static final int backRightMotorPort = 4;
    public static final int driveCurrentLimit = 40;
    public static final double trackWidth = 0.617;
    public static final double wheelDiameter = 0.1524;
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double maxVelocity = 3.0;
    public static final double maxAcceleration = 3.0;
    public static final double maxCentripetalAcceleration = 3.0;
    public static final double maxVoltage = 10.0;
    public static final double maxAngularVelocity = 2 * Math.PI;

    // Arm
    public static final int armMotorPort = 5;
    public static final int armSolenoidPortA = 13;
    public static final int armSolenoidPortB = 12;
    public static final int armSolenoidPortC = 11;
    public static final int armSolenoidPortD = 10;
    public static final double armP = 1.0;
    public static final double armI = 0.0;
    public static final double armD = 0.0;
    public static final double setpoint = 0.5;
    public static final double defaultPosition = 0.0;
    public static final int armCurrentLimit = 80;
    public static final double armMaxSpeed = 0.5;

    // Manipulator
    public static final int manipulatorSolenoidPortA = 14;
    public static final int manipulatorSolenoidPortB = 15;

    // Controllers
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
    public static final double deadband = 0.1;
    public static final double exponent = 0.0;

    // Vision
    public static final double visionCameraAngle = 0.0;
    public static final double visionCameraHeight = 15.13 - 26.5; // target - camera
    public static final double targetDistance = 18;
    public static final double targetHeight = 15.13;
    public static final double maxTurnSpeed = 0.2;
    public static final double minTurnAngle = 0.3;
    public static final int frontCameraPort = 0;
    public static final int rearCameraPort = 1;
    
    // Pneumatics
    public static final int compressorPort = 0;
    public static final PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
    public static final int hubPort = 1;

    // Auto
    public static final double autoDriveSpeed = 0.5;

}
