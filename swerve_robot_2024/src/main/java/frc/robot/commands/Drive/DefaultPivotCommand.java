package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class DefaultPivotCommand extends CommandBase {
    private PivotSubsystem m_Subsystem;

    public DefaultPivotCommand(PivotSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_Subsystem.ManualControl();

        // if(InputSystem.FaceLeft()){
            // CommandScheduler.getInstance().schedule(new RotateToCommand(Robot.d));
        // }
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
