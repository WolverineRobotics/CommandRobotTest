package frc.robot.commands.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.InputSystem;

public class ForwardDriveCommand extends CommandBase {
    private DriveSubsystem m_Subsystem;
    private boolean fini = false;
    private double distance;
    private double speed = InputSystem.DriveSpeed();

    public ForwardDriveCommand(DriveSubsystem subsystem, double _distance, double _speed){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
        _distance = distance;
        _speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override 
    public void execute() {
        m_Subsystem.MoveStraight(speed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return fini;
    }
}
