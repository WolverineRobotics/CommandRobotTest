package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InputSystem;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax motor;

    public IntakeSubsystem(){
        motor = new CANSparkMax(0, MotorType.kBrushless);
    }

    public void ManualControl(){
        motor.set(InputSystem.ManualIntake());
    }
    
}
