package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem m_Subsystem;

    public DefaultDriveCommand(DriveSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_Subsystem.ArcadeDrive();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
