package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class Arm {
    private CANSparkMax arm;
    private PIDController pid;

    public Arm() {
        this.arm = new CANSparkMax(Constants.armMotorPort, MotorType.kBrushless);
        this.arm.restoreFactoryDefaults();
        this.arm.setInverted(true);
        this.arm.setIdleMode(IdleMode.kBrake);
        this.arm.setSmartCurrentLimit(Constants.armCurrentLimit);

        this.pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);

        SmartDashboard.putNumber("Arm Current Limit", Constants.armCurrentLimit);
    }

    public void updateArm(double speed) {
        int limit = (int) Math.round(SmartDashboard.getNumber("Arm Current Limit", Constants.armCurrentLimit));
        arm.setSmartCurrentLimit(limit);

        this.arm.set(speed);
    }

    public void updateArmPID(double target) {
        double output = this.pid.calculate(this.arm.getEncoder().getPosition(), target);
        this.arm.set(output);
    }

    public void resetEncoder() {
        this.arm.getEncoder().setPosition(0);
    }

    public double getEncoderPosition() {
        return this.arm.getEncoder().getPosition();
    }

    public double getEncoderVelocity() {
        return this.arm.getEncoder().getVelocity();
    }
}
