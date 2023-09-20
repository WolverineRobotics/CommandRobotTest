package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.InputSystem;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private DriveSubsystem m_Subsystem;

    public DefaultDriveCommand(DriveSubsystem subsystem){
        m_Subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_Subsystem.ArcadeDrive();

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
