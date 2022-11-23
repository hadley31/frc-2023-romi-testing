package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class SetPose extends CommandBase {
    private final RomiDrivetrain m_drive;
    private final Pose2d m_pose;

    public SetPose(RomiDrivetrain drive, Pose2d pose) {
        m_drive = drive;
        m_pose = pose;
    }

    @Override
    public void execute() {
        System.out.println("Setting robot pose to " + m_pose);
        m_drive.resetPose(m_pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
