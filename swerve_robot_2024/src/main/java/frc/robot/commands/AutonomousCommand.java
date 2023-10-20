
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Robot;


public class AutonomousCommand extends CommandBase {

    DriveSubsystem weBalling;
    public AutonomousCommand(DriveSubsystem subsystem){
        weBalling = subsystem; // Initalize 'weBalling as a subsystem'
        addRequirements(weBalling); // Subsystem Dependency
    }

    @Override
    public void initialize(){
        withTimeout(0.5);
    }

    @Override
    public void execute(){
        weBalling.ArcadeDrive();
    }

    // Initialized when command stops
    @Override 
    public void end(boolean interrupted){ 
    }  

    // Will return true if finished
    @Override
    public boolean isFinished() { 
        return true;
    }
}
