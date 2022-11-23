package frc.robot.commands.debug;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PutSmartDashboardValue extends CommandBase {
    private final String m_key;
    private final String m_value;

    public PutSmartDashboardValue(String key, String value) {
        m_key = key;
        m_value = value;
    }

    @Override
    public void execute() {
        SmartDashboard.putString(m_key, m_value);
    }
}
