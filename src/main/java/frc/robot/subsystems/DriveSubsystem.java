package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
}
