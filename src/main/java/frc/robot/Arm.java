package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
    private CANSparkMax arm;
    private SparkMaxPIDController pid;
    private double speed;

    public Arm() {
        this.arm = new CANSparkMax(Constants.armMotorPort, MotorType.kBrushless);
        this.arm.restoreFactoryDefaults();
        this.arm.setInverted(true);
        this.arm.setIdleMode(IdleMode.kBrake);
        this.arm.setSmartCurrentLimit(Constants.armCurrentLimit);

        this.pid = arm.getPIDController();
        this.pid.setOutputRange(0, 1);
        this.pid.setP(Constants.armP);
        this.pid.setI(Constants.armI);
        this.pid.setD(Constants.armD);

        SmartDashboard.putNumber("Arm Current Limit", Constants.armCurrentLimit);
    }

    public void updateArm(double speed) {
        int limit = (int) Math.round(SmartDashboard.getNumber("Arm Current Limit", Constants.armCurrentLimit));
        arm.setSmartCurrentLimit(limit);

        if (Math.abs(speed) > Constants.armMaxSpeed) {
            this.speed = .5;
        } else if (Math.abs(speed) <= Constants.armMaxSpeed) {
            this.speed = speed;
        }

        this.arm.set(this.speed);
    }

    public void updateArmPID(double target) {
        this.pid.setReference(target, ControlType.kPosition);
    }

    public void resetArm() {
        this.pid.setReference(Constants.defaultPosition, ControlType.kPosition);
    }

    public void resetEncoder() {
        this.arm.getEncoder().setPosition(0);
    }

    public double getEncoderPosition() {
        return (this.arm.getEncoder().getPosition() * this.arm.getEncoder().getPositionConversionFactor());
    }

    public double getEncoderVelocity() {
        return this.arm.getEncoder().getVelocity();
    }
}
