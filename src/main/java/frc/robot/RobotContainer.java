// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CameraTurn;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.SingleJoystickDrive;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.subsystems.RomiCamera;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final RomiCamera m_camera = new RomiCamera(Constants.kCameraName);
    private final RomiDrivetrain m_drive = new RomiDrivetrain();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        XboxController controller = new XboxController(0);

        DriveWithJoystick driveCommand = new DriveWithJoystick(m_drive, () -> -controller.getLeftY(), () -> -controller.getRightX());
        Supplier<Double> speedSupplier = () -> controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
        SingleJoystickDrive singleJoystickDriveCommand = new SingleJoystickDrive(m_drive, speedSupplier,
                controller::getLeftX,
                controller::getLeftY, controller::getBButton);
        CameraTurn cameraTurnCommand = new CameraTurn(m_drive, m_camera);

        m_drive.setDefaultCommand(driveCommand);

        new Trigger(() -> controller.getXButton()).onTrue(new InstantCommand(() -> m_drive.resetPoseToCameraEstimate(), m_drive));
    }

    public void configureTeleopDefaultCommands() {
        System.out.println("Configuring teleop default commands");
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return TrajectoryFactory.getTrajectoryCommand(m_drive);
        return AutoFactory.testAuto(m_drive);
    }
}
