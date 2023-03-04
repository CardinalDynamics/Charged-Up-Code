// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // private final CANSparkMax m_left1 = new CANSparkMax(1, MotorType.kBrushless);
  // private final CANSparkMax m_left2 = new CANSparkMax(2, MotorType.kBrushless);
  // private final CANSparkMax m_right1 = new CANSparkMax(3, MotorType.kBrushless);
  // private final CANSparkMax m_right2 = new CANSparkMax(4, MotorType.kBrushless);
  private final Timer m_timer = new Timer();

  // private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);

  // private final DifferentialDrive m_drive = new DifferentialDrive(m_left1, m_right1);

  // private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // private final NetworkTableEntry tx = table.getEntry("tx");
  // private final NetworkTableEntry ty = table.getEntry("ty");
  // private final NetworkTableEntry ta = table.getEntry("ta");

  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_operator = new XboxController(1);

  // private final AnalogGyro gyro = new AnalogGyro(0);
  private AHRS navx;

  // private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  // private final DoubleSolenoid arm1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  // private final DoubleSolenoid arm2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
  // private final DoubleSolenoid manip1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);

  private boolean driveMode;
  private boolean inverted;

  private TankDrive drive;
  private Pneumatics pneumatics;
  private Arm arm;

  private SlewRateLimiter rateLimit1 = new SlewRateLimiter(.65);
  private SlewRateLimiter rateLimit2 = new SlewRateLimiter(.65);

  private final SendableChooser<String> autoOn = new SendableChooser<>();
  private String auto;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    autoOn.setDefaultOption("Auto On", "Auto On");
    autoOn.addOption("Middle Auto", "Middle Auto");
    autoOn.addOption("Auto Off", "Auto Off");
    SmartDashboard.putData("Auto On", autoOn);

    navx = new AHRS();

    drive = new TankDrive();
    pneumatics = new Pneumatics();
    arm = new Arm();

    // CameraServer.startAutomaticCapture("drive", 0);
    // CameraServer.startAutomaticCapture("manipulator", 1);

    // compressor.enableDigital();

    driveMode = true;
    inverted = false;

    pneumatics.setArm1(Value.kReverse);
    pneumatics.setArm2(Value.kReverse);
    pneumatics.setManipulator(Value.kForward);

    // gyro.reset();
    // gyro.calibrate();

    navx.reset();
    navx.calibrate();

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
    
    // SmartDashboard.putBooleanArray("Solenoid States", pneumatics.solenoidStates());

    if (m_controller.getAButtonPressed()) {
      driveMode =! driveMode;
    }
    if (m_controller.getXButtonPressed()) {
      inverted =! inverted;
    }

    SmartDashboard.putBoolean("Drive Mode", driveMode);
    SmartDashboard.putBoolean("Inverted", inverted);
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
    auto = autoOn.getSelected();
    auto = SmartDashboard.getString("Auto On", auto);
    // System.out.println("Auto selected: " + m_autoSelected);
    m_timer.reset();
    m_timer.start();
    pneumatics.setManipulator(Value.kReverse);

    // gyro.reset();
    navx.reset();
    
    drive.setBrakeMode(true);

    // auto = autoOn.getSelected();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // if (m_timer.get() < 1) {
    //   pneumatics.setManipulator(Value.kForward);
    // }
    // if (m_timer.get() < 1.25 && m_timer.get() > 1) {
    //   drive.updateSpeedArcade(.1, 0);
    // }
    // if (m_timer.get() < 2 && m_timer.get() > 1.25) {
    //   drive.updateSpeedArcade(-.25, 0);
    // }

    // switch (auto) {
    //   case "Auto On": {
        if (m_timer.get() < 1 && m_timer.get() > 0) {
          drive.updateSpeedArcade(-.3, 0);
        } else if (m_timer.get() < 2 && m_timer.get() > 1) {
          drive.updateSpeedArcade(.6, 0);
        } else if (m_timer.get() < 5.5  && m_timer.get() > 2) {
          drive.updateSpeedArcade(-.5, 0); }
      // }
      // case "Middle Auto": {
      //   if (m_timer.get() < 2 && m_timer.get() > 0) {
      //     drive.updateSpeedArcade(.5, 0);
      //   }
        // } else if (m_timer.get() > 2 && m_timer.get() < 15) {
        //   if (Math.abs(navx.getPitch()) < 10 && Math.abs(navx.getRoll()) < 10) {
        //     drive.updateSpeedArcade(0, 0);
        //   } else if (navx.getPitch() < -10) {
        //     drive.updateSpeedArcade(-.2, 0);
        //   } else if (navx.getPitch() > 10) {
        //     drive.updateSpeedArcade(.2, 0);
        //   }
        // }
      // }
      // case "Auto Off": {
      //   drive.updateSpeedArcade(0, 0);
      // }
  }
// }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_timer.stop();
    drive.setBrakeMode(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    if (driveMode == true) {
      double speed = rateLimit1.calculate(m_controller.getLeftY());
      double turn = m_controller.getRightX();
      if (inverted == true) {
        speed = -speed;
        turn = -turn;
        drive.updateSpeedArcade(speed, turn);
      }
      drive.updateSpeedArcade(speed, turn);
    } else if (driveMode == false) {
      double leftSpeed = rateLimit1.calculate(m_controller.getLeftY());
      double rightSpeed = rateLimit2.calculate(m_controller.getRightY());
      if (inverted == true) {
        leftSpeed = -rightSpeed;
        rightSpeed = -leftSpeed;
        drive.updateSpeedTank(leftSpeed, rightSpeed);
      }
      drive.updateSpeedTank(leftSpeed, rightSpeed);
    }

    // if (m_operator.getLeftTriggerAxis() > 0.05) {
    //   m_arm.setVoltage(-2);
    // } else if (m_operator.getRightTriggerAxis() > 0.05) {
    //   m_arm.setVoltage(4);
    // } else {
    //   m_arm.setVoltage(0.55);
    // }

    arm.updateArm(m_operator.getRightTriggerAxis());
    if (m_operator.getRightTriggerAxis() == 0) {
      arm.updateArm(-m_operator.getLeftTriggerAxis());
    }
    
    if (m_operator.getAButtonPressed()) {
      pneumatics.toggleArm1();
    }
    // if (m_operator.getBButtonPressed()) {
    //   pneumatics.toggleArm2();
    // }
    if (m_operator.getXButtonPressed()) {
      pneumatics.toggleManipulator();
    }
    if (m_operator.getYButtonPressed()) {
      pneumatics.toggleArm1();
      // pneumatics.toggleArm2();
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
