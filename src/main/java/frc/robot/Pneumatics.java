package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pneumatics {
    private DoubleSolenoid arm1;
    private DoubleSolenoid arm2;
    private DoubleSolenoid manipulator;
    private PneumaticHub pneumaticHub;
    private Compressor compressor;

    public Pneumatics() {
        this.arm1 = new DoubleSolenoid(Constants.moduleType, Constants.armSolenoidPortA, Constants.armSolenoidPortB);
        this.arm2 = new DoubleSolenoid(Constants.moduleType, Constants.armSolenoidPortC, Constants.armSolenoidPortD);
        this.manipulator = new DoubleSolenoid(Constants.moduleType, Constants.manipulatorSolenoidPortA, Constants.manipulatorSolenoidPortB);
        this.pneumaticHub = new PneumaticHub(Constants.hubPort);
        this.compressor = new Compressor(Constants.moduleType);

        compressor.enableDigital();

        SmartDashboard.putBoolean("Compressor Enabled", pneumaticHub.getCompressor());
        SmartDashboard.putBoolean("Compressor Pressure Switch", pneumaticHub.getPressureSwitch());
    }

    public void setArm1(Value value) {
        arm1.set(value);
    }

    public void setArm2(Value value) {
        arm2.set(value);
    }

    public void setManipulator(Value value) {
        manipulator.set(value);
    }

    
}
