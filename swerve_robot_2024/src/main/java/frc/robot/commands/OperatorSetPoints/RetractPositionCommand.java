package frc.robot.commands.OperatorSetPoints;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Elevator.DefaultElevatorCommand;
import frc.robot.commands.Intake.DefaultPivotCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class RetractPositionCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private PivotSubsystem pivot;

    public RetractPositionCommand(PivotSubsystem _pivot, ElevatorSubsystem _elevator){
        elevator = _elevator;
        pivot = _pivot;

        addRequirements(_pivot);
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {

        pivot.enable();
        elevator.enable();

        pivot.setGoal(0);
        elevator.setGoal(0);
    }
    
    @Override
    public void execute() { }
    @Override
    public void end(boolean interrupted) { 
        
        //pivot.disable();

        //if(!interrupted){
            //    CommandScheduler.getInstance().schedule(new DefaultPivotCommand(pivot));
            //    CommandScheduler.getInstance().schedule(new DefaultElevatorCommand(elevator));
            //}
    }

    @Override
    public boolean isFinished() {
        return (pivot.getController().atSetpoint() && elevator.getController().atSetpoint());
    }   
}