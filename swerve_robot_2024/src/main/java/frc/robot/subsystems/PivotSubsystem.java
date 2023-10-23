package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InputSystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.DefaultPivotCommand;

public class PivotSubsystem extends ProfiledPIDSubsystem{
    
    private CANSparkMax m_motor;// = new CANSparkMax(15, MotorType.kBrushless);
    private RelativeEncoder encoder;// = m_motor.getEncoder(); 

    public PivotSubsystem(){
        super(new ProfiledPIDController(0.1, 0, 0.0,
        new TrapezoidProfile.Constraints(OperatorConstants.kMaxPivotVelocity,OperatorConstants.kMaxPivotAcceleration)), 
        0);

        m_motor = new CANSparkMax(OperatorConstants.kPivotMotor, MotorType.kBrushless);
        encoder = m_motor.getEncoder();
        //encoder.setInverted(true);

        encoder.setPosition(0);
        setGoal(-10);
        getController().setTolerance(1);

        setDefaultCommand(new DefaultPivotCommand(this));
    }

    //public void OnStart(){
    //    encoder.setInverted(true);
//
    //}

    @Override
    public void periodic(){
        SmartDashboard.putNumber("pivot_encoder", encoder.getPosition());
        SmartDashboard.putNumber("pivot_error", getMeasurement());
        
        SmartDashboard.putBoolean("pivot_enabled", isEnabled());
        

        super.periodic();
    } 

    public void ManualControl(){
        double speed = InputSystem.ManualPivot();
        
        if(Math.abs(speed) >= 0.2 && !isEnabled()){ 
            //double inv_val = 0;
            //if(speed > 0){
            //    inv_val = 0.06;
            //}
            //else{
            //    inv_val = 0;
            //}
            //m_motor.set(speed * (0.35 + inv_val));
            m_motor.set(speed * (0.35));
        }
        else if(!isEnabled()){
            m_motor.set(0);
        }
    } 

    protected void useOutput(double output, TrapezoidProfile.State setpoint){
        SmartDashboard.putNumber("pivot_output", output);
        SmartDashboard.putNumber("pivot setpoint", setpoint.position);

        if(!getController().atSetpoint()){
            m_motor.set(output);
        }
        else{
            m_motor.set(0);
        }
    };
    
    protected double getMeasurement(){
        return encoder.getPosition();
    };
}
