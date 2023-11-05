package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.InputSystem;
import frc.robot.subsystems.DriveSubsystem;

public class ForwardDriveCommand extends CommandBase {

    private DriveSubsystem m_Subsystem;
    private double speed, time, target;
    
    // SPEED VALUES ABOVE 0 DRIVE IN THE DIRECTION OF THE NO PARKING SIGN
    // NEGATIVE SPEED VALUES DRIVE TOWARDS THE BATTERY SIDE
    public ForwardDriveCommand(DriveSubsystem subsystem, double _speed, double _time){  
        m_Subsystem = subsystem;
        addRequirements(subsystem);
        speed = _speed;
        time = _time;
    }

    @Override
    public void initialize() {
        //target = m_Subsystem.getPigeonHeading();
    }
    
    @Override
    public void execute() {
        m_Subsystem.MoveStraight(speed);
        time -= 20;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Subsystem.MoveStraight(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return (time <= 0);
    }
}
