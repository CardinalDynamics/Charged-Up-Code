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
    private Value[] solenoidStates;
    private boolean[] solenoidReturn;

    public Pneumatics() {
        this.arm1 = new DoubleSolenoid(Constants.moduleType, Constants.armSolenoidPortA, Constants.armSolenoidPortB);
        this.arm2 = new DoubleSolenoid(Constants.moduleType, Constants.armSolenoidPortC, Constants.armSolenoidPortD);
        this.manipulator = new DoubleSolenoid(Constants.moduleType, Constants.manipulatorSolenoidPortA, Constants.manipulatorSolenoidPortB);
        this.pneumaticHub = new PneumaticHub(Constants.hubPort);
        this.compressor = new Compressor(Constants.moduleType);

        compressor.enableDigital();
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

    public void toggleArm1() {
        arm1.toggle();
    }

    public void toggleArm2() {
        arm2.toggle();
    }

    public void toggleManipulator() {
        manipulator.toggle();
    }

    public void updateDashboard() {
        SmartDashboard.putBoolean("Pressure Switch", pneumaticHub.getPressureSwitch());
        SmartDashboard.putBoolean("Compressor Enabled", compressor.isEnabled());
    }

    // public boolean[] solenoidStates() {
    //     solenoidStates[0] = arm1.get();
    //     solenoidStates[1] = arm2.get();
    //     solenoidStates[2] = manipulator.get();
    //     solenoidReturn[0] = toBool(solenoidStates[0]);
    //     solenoidReturn[1] = toBool(solenoidStates[1]);
    //     solenoidReturn[2] = toBool(solenoidStates[2]);
    //     return solenoidReturn;
    // }

    // private boolean toBool(Value value) {
    //     if (value == Value.kForward) {
    //         return true;
    //     }
    //     return false;
    // }
}
