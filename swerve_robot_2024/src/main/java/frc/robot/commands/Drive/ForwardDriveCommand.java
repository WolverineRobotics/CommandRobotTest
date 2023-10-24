package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.InputSystem;
import frc.robot.subsystems.DriveSubsystem;

public class ForwardDriveCommand extends CommandBase {

    private DriveSubsystem m_Subsystem;
    private double speed, time, target;

    public ForwardDriveCommand(DriveSubsystem subsystem, double _speed, double _time){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
        speed = _speed;
        time = _time;
    }

    @Override
    public void initialize() {
        target = m_Subsystem.getPigeonHeading();
    }
    
    @Override
    public void execute() {
        m_Subsystem.MoveStraightPropotionate(speed, target);
        time -= 20;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(time <= 0){
            m_Subsystem.MoveStraightPropotionate(0, target);
        }
      return (time <= 0);
    }
}
