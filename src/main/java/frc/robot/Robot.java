// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // public Drive drive;
  public Vision vision;

  // Motor Controllers
  private final CANSparkMax m_left1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_left2 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_right1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_right2 = new CANSparkMax(4, MotorType.kBrushless);

  // left and right side encoders
  private final RelativeEncoder m_leftEncoder = m_left1.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_right1.getEncoder();

  // arm controller
  private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);

  // differential drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_left1, m_right1);

  // odometry
  // private final DifferentialDriveOdometry m_odometry;

  // limelight stuff
  // private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // private final NetworkTableEntry tx = table.getEntry("tx");
  // private final NetworkTableEntry ty = table.getEntry("ty");
  // private final NetworkTableEntry ta = table.getEntry("ta");

  // controllers
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_operator = new XboxController(1);

  // pneumatics
  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private final DoubleSolenoid arm1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10, 11);
  private final DoubleSolenoid arm2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 12, 13);
  private final DoubleSolenoid manip = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);
  private final PneumaticHub hub = new PneumaticHub();

  // gyros
  // private final AHRS navx = new AHRS(Port.kMXP);
  private final AnalogGyro gyro = new AnalogGyro(0);

  private boolean driveMode;
  private boolean armHold;

  private double armVoltage;
  private double armMotor;

  // rate limiters
  private SlewRateLimiter rateLimit1 = new SlewRateLimiter(1);
  private SlewRateLimiter rateLimit2 = new SlewRateLimiter(1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // drive = new Drive();
    vision = new Vision();



    // set up motors
    m_left1.restoreFactoryDefaults();
    m_left2.restoreFactoryDefaults();
    m_right1.restoreFactoryDefaults();
    m_right2.restoreFactoryDefaults();
    m_arm.restoreFactoryDefaults();

    // set motor idle mode
    m_left1.setIdleMode(IdleMode.kCoast);
    m_left2.setIdleMode(IdleMode.kCoast);
    m_right1.setIdleMode(IdleMode.kCoast);
    m_right2.setIdleMode(IdleMode.kCoast);
    m_arm.setIdleMode(IdleMode.kBrake);

    m_left1.setSmartCurrentLimit(80);
    m_left2.setSmartCurrentLimit(80);
    m_right1.setSmartCurrentLimit(80);
    m_right2.setSmartCurrentLimit(80);
    m_arm.setSmartCurrentLimit(80);

    m_left2.follow(m_left1);
    m_right2.follow(m_right1);

    m_right1.setInverted(true);
    m_right2.setInverted(true);
    m_arm.setInverted(true);

    gyro.reset();
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    

    CameraServer.startAutomaticCapture("drive", 0);
    CameraServer.startAutomaticCapture("manipulator", 1);

    compressor.enableDigital();

    driveMode = true;
    armHold = false;

    // m_odometry = new DifferentialDriveOdometry();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);

    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);

    // SmartDashboard.putNumber("NavX", navx.getAngle());
    SmartDashboard.putNumber("Gyro", gyro.getAngle());

    armVoltage = (m_arm.getBusVoltage() * m_arm.getAppliedOutput());
    SmartDashboard.putNumber("Arm Voltage", armVoltage);
    SmartDashboard.putNumber("arm motor", armMotor);

    SmartDashboard.putBoolean("Drive Mode", driveMode);
    SmartDashboard.putBoolean("Arm Hold", armHold);
    SmartDashboard.putBoolean("Pressure Switch", hub.getPressureSwitch());
    SmartDashboard.putNumber("Compressor Current", hub.getCompressorCurrent());

    if (m_controller.getAButtonPressed()) {
      driveMode =! driveMode;
    }

    if (m_operator.getLeftStickButtonPressed()) {
      armHold =! armHold;
    }

    SmartDashboard.putBoolean("Drive Mode", driveMode);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    // m_odometry.resetPosition(gyro.getAngle(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    double dist = vision.estimateDistance();
    
    
    if (driveMode == true) {
      m_drive.arcadeDrive(-rateLimit1.calculate(m_controller.getLeftY()), m_controller.getLeftX());
    } else if (driveMode == false) {
      m_drive.tankDrive(-rateLimit1.calculate(m_controller.getLeftY()), -rateLimit2.calculate(m_controller.getRightY()));
    }

    // if (m_operator.getLeftBumper()) {
    //   m_arm.setVoltage(-2);
    // } else if (m_operator.getRightBumper()) {
    //   m_arm.setVoltage(1);
    // } else if (armHold == true) {
    //   m_arm.setVoltage(0.5);
    // } else if (armHold == false) {
    //   m_arm.setVoltage(0);
    // }
    m_arm.setVoltage(1);
    
    // Prototype code for arm: A and B will extend both arm pistons
    if (m_operator.getAButton()) {
      arm1.set(Value.kForward);
      arm2.set(Value.kForward);
    } else if (m_operator.getBButton()) {
      arm1.set(Value.kReverse);
      arm2.set(Value.kReverse);
    }

    // Prototype code for manipulator: X will extend, Y will retract
    if (m_operator.getXButton()) {
      manip.set(Value.kForward);
    } else if (m_operator.getYButton()) {
      manip.set(Value.kReverse);
    }

    if (m_operator.getRightStickButtonPressed()) {
      arm1.set(Value.kOff);
      arm2.set(Value.kOff);
      manip.set(Value.kOff);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
