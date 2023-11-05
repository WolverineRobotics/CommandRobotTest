package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends CommandBase {
    private IntakeSubsystem m_Subsystem;
    private double speed;
    private double time;

    public RunIntakeCommand(IntakeSubsystem subsystem, double _speed){
        m_Subsystem = subsystem;
        speed = _speed;
        addRequirements(subsystem);
        time = 750;
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_Subsystem.Run(speed);
        time -=20;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Subsystem.Run(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (time <= 0);
    }  
}