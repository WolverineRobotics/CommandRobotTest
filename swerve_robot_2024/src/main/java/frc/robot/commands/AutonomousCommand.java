
package.robot.frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutonomousCommand extends CommandBase {

    DriveSubsystem weBalling;
    public AutoCommand(weBalling subsystem){
        weBalling = subsystem; // Initalize 'weBalling as a subsystem'
        addRequirements(weBalling); // Subsystem Dependency

        timer = new Timer(); 
    }
    
    // Command for Driving 
    @Override
    public void intitialize(){
        Timer.reset();
        Timer.start();
    }

    @Override
    public void execute(){
        weBalling.DifferentialDrive(.5, 0); // Half speed, drive straight
    }

    // Initialized when command stops
    @Override 
    public void end(boolean interrupted){ 
    }  

    // Will return true if finished
    @Override
    public boolean isFinished() { 
        return timer.get() > 2;
    }
}