package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase{

    private DriveSubsystem m_subsystem;

    private double offset = -5;
    private double balanced_range = 2.5;
    private double speed = 0.275;//, target;

    private boolean fini = false;

    public AutoBalance(DriveSubsystem _subsystem){
        m_subsystem = _subsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        //target = m_subsystem.getPigeonHeading();
    }


    @Override
    public void execute(){
        double angle = m_subsystem.Pitch() + offset;

        if(angle > balanced_range){ m_subsystem.MoveStraight(-speed); }
        else if(angle < -balanced_range){ m_subsystem.MoveStraight(speed); }

        else{ m_subsystem.MoveStraight(0); }
    }

    public void EndCall(){
        fini = true;
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return fini;
    }
    
}
