package frc.robot.commands.Drive;

import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToCommand extends CommandBase {

    private final DriveSubsystem m_Subsystem;
    private double angle;

    private double deadzone = 2;
    private double speed;
    private double gain = 0.01;
    private double error;

    private int dir = 0;
    
    private boolean fini = false;

    public RotateToCommand(DriveSubsystem subsystem, double _angle){
        m_Subsystem = subsystem;
        angle = _angle;
        error = angle - m_Subsystem.Yaw();
        addRequirements(subsystem);
    }

    

    @Override
    public void initialize() {
        if(m_Subsystem.Yaw() > angle){ dir = 1; }
        else{ dir = -1; }

    }
    
    @Override
    public void execute() {
        // Proportional Control
        error = angle - m_Subsystem.Yaw();
        speed = gain * error;

        //Actual rotate fucntion
        m_Subsystem.Rotate(speed * dir);

        if(error < deadzone && error > -deadzone){
            fini = true;
        }
    
    }

    public void EndCall(){
        fini = true;
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return fini;
    }


    
}