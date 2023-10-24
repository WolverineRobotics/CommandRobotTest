package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InputSystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.DefaultIntakeCommand;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax motor;

    public IntakeSubsystem(){
        motor = new CANSparkMax(OperatorConstants.kIntakeMotor, MotorType.kBrushless);
        setDefaultCommand(new DefaultIntakeCommand(this));
    }

    public void ManualControl(){
        motor.set(InputSystem.ManualIntake());
    }

    public void Run(double speed){
        motor.set(speed);
    }
    
}
