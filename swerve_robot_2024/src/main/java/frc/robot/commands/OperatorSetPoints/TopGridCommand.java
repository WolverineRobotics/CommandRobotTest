package frc.robot.commands.OperatorSetPoints;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class TopGridCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private PivotSubsystem pivot;

    public TopGridCommand(PivotSubsystem _pivot, ElevatorSubsystem _elevator){
        elevator = _elevator;
        pivot = _pivot;

        addRequirements(_pivot);
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        pivot.setSetpoint(-20);
        elevator.setSetpoint(-45);
    }
    
    @Override
    public void execute() { }
    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return (pivot.getController().atSetpoint() && elevator.getController().atSetpoint());
    }   
}