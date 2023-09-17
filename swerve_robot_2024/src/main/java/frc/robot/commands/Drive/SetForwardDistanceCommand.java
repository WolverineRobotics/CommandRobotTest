package frc.robot.commands.Drive;
import com.ctre.phoenix.sensors.PigeonIMU;

public class SetForwardDistanceCommand extends CommandBase {
    PigeonIMU penisBalls = new PigeonIMU(0);

    private final SetForwardDistanceSubsystem m_Subsystem;

    public SetForwardDistanceCommand(DriveSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }
}
