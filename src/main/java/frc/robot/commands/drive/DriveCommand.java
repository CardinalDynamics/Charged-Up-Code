package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier driveY;
    private final DoubleSupplier driveX;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier driveY, DoubleSupplier driveX) {
        this.driveSubsystem = driveSubsystem;
        this.driveY = driveY;
        this.driveX = driveX;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double xSpeed = driveY.getAsDouble();
        double zRotation = driveX.getAsDouble();
        if (Math.abs(xSpeed) < Constants.OperatorConstants.yGate) xSpeed = 0;
        if (Math.abs(zRotation) < Constants.OperatorConstants.xGate) zRotation = 0;

        driveSubsystem.arcadeDrive(driveY.getAsDouble(), driveX.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.arcadeDrive(0, 0);
    }
}
