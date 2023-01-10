// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriveControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double yGate = 0.15;
    public static final double xGate = 0.15;
  }

  public static class DriveConstants {
    public static final int kFrontLeftMotorPort = 0;
    public static final int kFrontRightMotorPort = 1;
    public static final int kBackLeftMotorPort = 2;
    public static final int kBackRightMotorPort = 3;
    
    public static final int kTrackWidth = 1;

    public static final double maxVelocity = 1;
    public static final double maxAcceleration = 1;

    // Ramsete constants
    public static final double b = 2;
    public static final double zeta = 0.7;

    // drivebase tuning
    public static final double ksVolts = 0.16302;
    public static final double kvVoltSecondsPerMeter = 2.8008; //7.2507
    public static final double kaVoltSecondsSquaredPerMeter = 0.36623;
    public static final double kPDriveVel = 0;// 3.5796
  }

  public static class VisionConstants {
    public static final int pGain = 16;

    public static final double VisX_kP = 1;
    public static final double VisX_MAX = .3;
    public static final double VisTimeLimit = 5;
    public static final double VisX_Offset = 0;
    public static final double VisX_Tol = 0;
    public static final double VisY_Tol = 5;
    public static final double VisY_distanceConstant = 254.3;
    public static final double VisY_Offset = 8;
    public static final double VisY_VTol = 100;
    public static final double VisX_VTol = 100;
    public static final int Vis_TimerConfidence = 5;
    public static double Vis_LLAngle = 35;
  }

  public static class Pneumatics {
    public static final PneumaticsModuleType compressor = PneumaticsModuleType.CTREPCM;
    public static final PneumaticsModuleType arm = PneumaticsModuleType.CTREPCM;
    public static final PneumaticsModuleType claw = PneumaticsModuleType.CTREPCM;
  }
}
