// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

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

  public TankDrive tankDrive;
  public Vision vision;
  public Arm arm;
  public Pneumatics pneumatics;
  public Autos autos;

  public double[] armPID;

  // controllers
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_operator = new XboxController(1);

  // gyros
  // private final AHRS navx = new AHRS(Port.kMXP);
  private final AnalogGyro gyro = new AnalogGyro(0);

  private boolean driveMode;
  private boolean armHold;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    tankDrive = new TankDrive();
    vision = new Vision();
    arm = new Arm();
    pneumatics = new Pneumatics();
    autos = new Autos();

    driveMode = true;
    armHold = false;

    gyro.calibrate();
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

    armPID = arm.getPIDValues();

    // SmartDashboard.putNumber("NavX", navx.getAngle());
    SmartDashboard.putNumber("Gyro", gyro.getAngle());

    SmartDashboard.putBoolean("Drive Mode", driveMode);
    SmartDashboard.putBoolean("Arm Hold", armHold);

    if (m_controller.getAButtonPressed()) {
      driveMode =! driveMode;
    }

    if (m_operator.getLeftStickButtonPressed()) {
      armHold =! armHold;
    }

    SmartDashboard.putNumber("Arm Angle", arm.getEncoderPosition());

    pneumatics.updateDashboard();
    arm.updateDashboard();

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

    autos.autoInit();

    // m_odometry.resetPosition(gyro.getAngle(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double timeFromAutoStart = 15.0 - Timer.getMatchTime();

    autos.autoPeriodic();

    if (timeFromAutoStart < 1.0) {
      // Go backwards
      tankDrive.updateSpeedTank(-Constants.autoDriveSpeed, -Constants.autoDriveSpeed);
    } else if (timeFromAutoStart < 2.0) {
      // Turn 180 Degrees
    }

    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    // }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    double dist = vision.estimateDistance();
    SmartDashboard.putBoolean("In Placement Range", (dist > 0 && dist < 20));
    
    if (driveMode == true) {
      double leftSpeed = joystickResponse(m_controller.getLeftY());
      double rightSpeed = joystickResponse(m_controller.getRightY());
      tankDrive.updateSpeedTank(leftSpeed, rightSpeed);
    } else if (driveMode == false) {
      double leftSpeed = joystickResponse(m_controller.getLeftY());
      double rightSpeed = joystickResponse(m_controller.getRightX());
      tankDrive.updateSpeedArcade(leftSpeed, rightSpeed);
    }

    if (armHold == true) {
      arm.updateArmPID(Constants.setpoint);
    } else if (armHold == false) {
      arm.updateArm(m_operator.getLeftTriggerAxis());
    } else if (m_operator.getLeftTriggerAxis() < .1) {
      arm.resetArm();
    }
    
    
    // Prototype code for arm: A and B will extend both arm pistons
    if (m_operator.getRightBumper()) {
      pneumatics.setArm1(Value.kForward);
      pneumatics.setArm2(Value.kReverse);
    } else if (m_operator.getLeftBumper()) {
      pneumatics.setArm1(Value.kReverse);
      pneumatics.setArm2(Value.kForward);
    }

    // Prototype code for manipulator: X will extend, Y will retract
    if (m_operator.getRightTriggerAxis() >= .1) {
      pneumatics.setManipulator(Value.kForward);
    } else if (m_operator.getRightTriggerAxis() < .1) {
      pneumatics.setManipulator(Value.kReverse);
    }

    if (m_operator.getRightStickButtonPressed()) {
      pneumatics.setArm1(Value.kOff);
      pneumatics.setArm2(Value.kOff);
      pneumatics.setManipulator(Value.kOff);
    }
  }

  // iron riders
  private double joystickResponse(double raw) {
    double deadband = SmartDashboard.getNumber("Deadband", Constants.deadband);
    double deadbanded = 0.0;
    if (raw > deadband) {
      deadbanded = raw - deadband;
    } else if (raw < -deadband) {
      deadbanded = raw + deadband;
    }
    double exponent = SmartDashboard.getNumber("Exponent", Constants.exponent) + 1;
    return Math.pow(Math.abs(deadbanded), exponent) * Math.signum(deadbanded);
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
