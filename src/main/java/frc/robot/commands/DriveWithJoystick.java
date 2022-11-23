package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveWithJoystick extends CommandBase {
    private RomiDrivetrain drive;
    private Supplier<Double> moveSpeedSupplier, turnSpeedSupplier;

    public DriveWithJoystick(RomiDrivetrain drive, Supplier<Double> moveSpeed, Supplier<Double> turnSpeed) {
        this.drive = drive;
        this.moveSpeedSupplier = moveSpeed;
        this.turnSpeedSupplier = turnSpeed;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double moveSpeed = moveSpeedSupplier.get();
        double turnSpeed = turnSpeedSupplier.get();

        drive.arcadeDrive(moveSpeed, turnSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        drive.brake();
    }
}
