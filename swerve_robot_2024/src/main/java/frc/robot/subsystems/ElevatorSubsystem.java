package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    
    private CANSparkMax m_motor_1, m_motor_2;
    private MotorControllerGroup m_motors;
    private RelativeEncoder encoder; 

    public ElevatorSubsystem(){
        m_motor_1 = new CANSparkMax(0, MotorType.kBrushless);
        m_motor_2 = new CANSparkMax(0, MotorType.kBrushless);
        m_motors = new MotorControllerGroup(m_motor_1, m_motor_2);
        encoder = m_motor_1.getEncoder();
        encoder.setPosition(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("elevator_encoder", encoder.getPosition());
    }
}
