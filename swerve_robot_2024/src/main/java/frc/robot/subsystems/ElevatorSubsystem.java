package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InputSystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Elevator.DefaultElevatorCommand;

public class ElevatorSubsystem extends PIDSubsystem{
    
    private CANSparkMax m_motor_1, m_motor_2;
    private MotorControllerGroup m_motors;
    private RelativeEncoder encoder; 
    private PIDController pid;

    public ElevatorSubsystem(){

        super(new PIDController(0.05, 0.01, 0.02));
        m_motor_1 = new CANSparkMax(OperatorConstants.kElevatorMotor1, MotorType.kBrushless);
        m_motor_2 = new CANSparkMax(OperatorConstants.kElevatorMotor2, MotorType.kBrushless);
        m_motors = new MotorControllerGroup(m_motor_1, m_motor_2);
        encoder = m_motor_1.getEncoder();
        //encoder.setInverted(true);
        encoder.setPosition(0);
        
        getController().setSetpoint(0);
        getController().setTolerance(1);

        setDefaultCommand(new DefaultElevatorCommand(this));

    }

    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("elevator_encoder", encoder.getPosition());
        
        if(InputSystem.Operator().getAButtonPressed()){
            enable();
        }
        if(InputSystem.Operator().getAButtonReleased()){
            disable();
        }
        
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
        protected void useOutput(double output, double setpoint){
            m_motors.set(output);
        };
    
        protected double getMeasurement(){
            return getSetpoint() - encoder.getPosition();
        };
}
