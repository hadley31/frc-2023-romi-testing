// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.util.GeometryUtils;

public class SingleJoystickDrive extends CommandBase {
    private final RomiDrivetrain m_drivetrain;
    private final Supplier<Double> m_speedSupplier;
    private final Supplier<Double> m_xSupplier;
    private final Supplier<Double> m_ySupplier;
    private final Supplier<Boolean> m_boostSupplier;

    private final double accel = 0.1;
    private double speed = 0;

    /**
     * Creates a new ArcadeDrive. This command will drive your robot according to
     * the speed supplier lambdas. This command does not terminate.
     *
     * @param drivetrain          The drivetrain subsystem on which this command
     *                            will run
     * @param xaxisSpeedSupplier  Lambda supplier of forward/backward speed
     * @param zaxisRotateSupplier Lambda supplier of rotational speed
     */
    public SingleJoystickDrive(RomiDrivetrain drivetrain, Supplier<Double> speedSupplier, Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Boolean> boostSupplier) {
        m_drivetrain = drivetrain;
        m_speedSupplier = speedSupplier;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_boostSupplier = boostSupplier;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Initializing joystick drive");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = m_xSupplier.get();
        double y = m_ySupplier.get();

        m_drivetrain.arcadeDrive(getForwardSpeed(x, y), getTurningSpeed(x, y));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.brake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private double getTurningSpeed(double x, double y) {
        Rotation2d angle = m_drivetrain.getPose().getRotation();
        Rotation2d desiredAngle = new Rotation2d(x, y);
        
        // Create a deadzone where we don't rotate
        if (x * x + y * y < 0.2) {
            return 0;
        }

        Rotation2d smallestAngle = GeometryUtils.angleBetween(angle, desiredAngle);

        if (Math.abs(smallestAngle.getDegrees()) > 172) {
            smallestAngle = Rotation2d.fromDegrees(Math.abs(smallestAngle.getDegrees()));
        }

        if (Math.abs(smallestAngle.getDegrees()) < 2.5) {
            return 0;
        }

        double logSmallestAngle = Math.log1p(Math.abs(smallestAngle.getDegrees()));

        return MathUtil.clamp(logSmallestAngle * Math.signum(smallestAngle.getDegrees()) * 0.15, -0.7, 0.7);
    }

    private static final double kZThreshold = Units.degreesToRadians(45.0);

    private double getForwardSpeed(double x, double y) {
        Rotation2d angle = m_drivetrain.getPose().getRotation();
        Rotation2d desiredAngle = new Rotation2d(x, y);

        Rotation2d smallestAngle = GeometryUtils.angleBetween(angle, desiredAngle);

        double trigger_input = m_speedSupplier.get();
        double accelMult = m_boostSupplier.get() ? 7 : 1;

        double desiredSpeed = 0;

        if (smallestAngle.getRadians() < kZThreshold) {
            double input = Math.sqrt(x * x + y * y);
            desiredSpeed = input * Math.log1p(Math.abs(input) * Math.E) * trigger_input * 0.5;
        } else {
            // desiredSpeed = trigger_input;
            desiredSpeed = 0;
        }

        System.out.println(desiredSpeed);

        speed = MathUtil.interpolate(speed, desiredSpeed, accel * accelMult);

        return speed;
    }
}
