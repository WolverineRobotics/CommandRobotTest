package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends CommandBase {
    private IntakeSubsystem m_Subsystem;

    public DefaultIntakeCommand(IntakeSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() { m_Subsystem.ManualControl(); }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }   
}
