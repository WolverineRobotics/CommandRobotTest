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
import frc.robot.commands.Intake.DefaultPivotCommand;

public class PivotSubsystem extends PIDSubsystem{
    
    private CANSparkMax m_motor;// = new CANSparkMax(15, MotorType.kBrushless);
    private RelativeEncoder encoder;// = m_motor.getEncoder(); 

    public PivotSubsystem(){
        super(new PIDController(0.1, 0, 0.0, 0.02), -10);

        m_motor = new CANSparkMax(OperatorConstants.kPivotMotor, MotorType.kBrushless);
        encoder = m_motor.getEncoder();
        //encoder.setInverted(true);
        encoder.setPosition(0);

        setDefaultCommand(new DefaultPivotCommand(this));

        setSetpoint(-10);
        getController().setTolerance(1);


    }

    //public void OnStart(){
    //    encoder.setInverted(true);
//
    //}

    @Override
    public void periodic(){
        SmartDashboard.putNumber("pivot_encoder", encoder.getPosition());
        SmartDashboard.putNumber("pivot_error", getMeasurement());
        SmartDashboard.putNumber("pivot setpoint", getSetpoint());
        SmartDashboard.putBoolean("pivot_enabled", isEnabled());

//
        //SmartDashboard.putNumber("pivot_p", getController().());
        //SmartDashboard.putNumber("pivot_i", getMeasurement());
        //SmartDashboard.putBoolean("pivot_d", ());
        

        //if (isEnabled()) {
        //    useOutput(getController().calculate(getMeasurement()), getSetpoint());
        //}

        super.periodic();
        //getController().calculate(getMeasurement());
    } 

    public void ManualControl(){
        
        double speed = InputSystem.ManualPivot();
        

        if(Math.abs(speed) >= 0.2 && !isEnabled()){ 
            double inv_val = 0;
            if(speed > 0){
                inv_val = 0.06;
            }
            else{
                inv_val = 0;
            }
            m_motor.set(speed * (0.35 + inv_val));

            //if(encoder.getPosition() > 48 && speed > 0 ){
            //    m_motor.set(0);
            //}
            //else if(encoder.getPosition() <= 2 && speed < 0){
            //    m_motor.set(0);
            //}
            //else{
            //    m_motor.set(speed * 0.35);
//
            //}
        }
        else if(!isEnabled()){
            m_motor.set(0);
        }
        

    } 

    protected void useOutput(double output, double setpoint){
        SmartDashboard.putNumber("pivot_output", output);

        m_motor.set(output);
    };
    
    protected double getMeasurement(){
        SmartDashboard.putNumber("pivot_measurment", getSetpoint() - encoder.getPosition());
        return encoder.getPosition();
    };
}
