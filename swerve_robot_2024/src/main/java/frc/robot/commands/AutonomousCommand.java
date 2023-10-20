
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class AutonomousCommand extends CommandBase {

    DriveSubsystem weBalling;
    public AutonomousCommand(DriveSubsystem subsystem){
        weBalling = subsystem; // Initalize 'weBalling as a subsystem'
        addRequirements(weBalling); // Subsystem Dependency

    }

    @Override
    public void intitialize(){
        withTimeout(0.5);
    }

    @Override
    public void execute(){
    //    weBalling.DifferentialDrive(.5, 0); // Half speed, drive straight
    }

    // Initialized when command stops
    @Override 
    public void end(boolean interrupted){ 
    }  

    // Will return true if finished
    @Override
    public boolean isFinished() { 
    //    return timer.get() > 2;
        return true;
    }
}
