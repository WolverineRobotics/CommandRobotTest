package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InputSystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Elevator.DefaultElevatorCommand;

public class ElevatorSubsystem extends ProfiledPIDSubsystem{
    
    private CANSparkMax m_motor_1, m_motor_2;
    private MotorControllerGroup m_motors;
    private RelativeEncoder encoder; 

    public ElevatorSubsystem(){

        super(
            new ProfiledPIDController(0.04, 0, 0, 
            new TrapezoidProfile.Constraints(OperatorConstants.kMaxElevatorVelocity,OperatorConstants.kMaxElevatorAcceleration)), 
            0
        );

        m_motor_1 = new CANSparkMax(OperatorConstants.kElevatorMotor1, MotorType.kBrushless);
        m_motor_2 = new CANSparkMax(OperatorConstants.kElevatorMotor2, MotorType.kBrushless);
        m_motors = new MotorControllerGroup(m_motor_1, m_motor_2);
        encoder = m_motor_1.getEncoder();
        //encoder.setInverted(true);
        
        encoder.setPosition(0);
        setGoal(-10);
        getController().setTolerance(1);

        setDefaultCommand(new DefaultElevatorCommand(this));

    }

    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("elevator_encoder", encoder.getPosition());
        SmartDashboard.putNumber("elevator_error", getController().getPositionError());
        SmartDashboard.putBoolean("elevator_enabled", isEnabled());       

        super.periodic();
    } 
    
    public void ManualControl(){
        double speed = InputSystem.ManualElevator();
        if(Math.abs(speed) >= 0.2){ 

            m_motors.set(speed * 0.35);
            
        }
        else if(!isEnabled()){
            m_motors.set(0);
        }
            
    } 
    protected void useOutput(double output, TrapezoidProfile.State setpoint){
        SmartDashboard.putNumber("elevator_output", output);
        SmartDashboard.putNumber("elevator_setpoint", setpoint.position);


        if(!getController().atSetpoint()){
            m_motors.set(output);
        }
        else{
            m_motors.set(0);
        }
    };

    protected double getMeasurement(){
        return encoder.getPosition();
    };
}
