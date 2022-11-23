package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiCamera;
import frc.robot.subsystems.RomiDrivetrain;

public class CameraTurn extends CommandBase {
    private static final double kSpeedMultiplier = 0.5;
    private static final double kMinimumSpeed = 0.2;
    private static final double kAngleThreshold = 1.0;
    private static final double kMaxAngle = 25.0;

    private RomiDrivetrain drive;
    private RomiCamera camera;

    public CameraTurn(RomiDrivetrain drive, RomiCamera camera) {
        this.drive = drive;
        this.camera = camera;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        var latestResult = camera.getLatestResult();
        double speed = 0;

        if (latestResult != null) {
            double angle = latestResult.getYaw();
            speed = calculateTurnSpeed(angle);
        }

        drive.arcadeDrive(0, speed);
    }

    @Override
    public void end(boolean interrupted) {
        drive.brake();
    }

    private double calculateTurnSpeed(double targetYawOffset) {
        if (Math.abs(targetYawOffset) < kAngleThreshold) {
            return 0;
        }

        double m = kSpeedMultiplier / kMaxAngle;
        double b = Math.copySign(kMinimumSpeed, targetYawOffset);

        return m * targetYawOffset + b;
    }
}
