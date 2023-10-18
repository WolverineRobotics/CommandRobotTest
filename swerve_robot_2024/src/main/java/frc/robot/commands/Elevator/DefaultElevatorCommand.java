package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends CommandBase {
    private ElevatorSubsystem m_Subsystem;

    public DefaultElevatorCommand(ElevatorSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() { m_Subsystem.ManualControl(); }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
    
}
