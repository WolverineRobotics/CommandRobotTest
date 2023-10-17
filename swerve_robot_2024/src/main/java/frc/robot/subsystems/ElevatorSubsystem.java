package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InputSystem;
import frc.robot.commands.Drive.DefaultElevatorCommand;

public class ElevatorSubsystem extends SubsystemBase{
    
    private CANSparkMax m_motor_1, m_motor_2;
    private MotorControllerGroup m_motors;
    private RelativeEncoder encoder; 

    public ElevatorSubsystem(){
        m_motor_1 = new CANSparkMax(10, MotorType.kBrushless);
        m_motor_2 = new CANSparkMax(14, MotorType.kBrushless);
        m_motors = new MotorControllerGroup(m_motor_1, m_motor_2);
        encoder = m_motor_1.getEncoder();
        //encoder.setInverted(true);
        encoder.setPosition(0);

        setDefaultCommand(new DefaultElevatorCommand(this));

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("elevator_encoder", encoder.getPosition());
        
    } 

    public void ManualControl(){
        double speed = InputSystem.ManualElevator();
        if(Math.abs(speed) >= 0.2){ 
             m_motors.set(speed * 0.35);

            //if(encoder.getPosition() > 48 || encoder.getPosition() <= 2 ){
            //    m_motors.set(0);
            //}
            //else{
            //    m_motors.set(speed * 0.35);
//
            //}
        }
        else{
            m_motors.set(0);
        }

    } 
}
