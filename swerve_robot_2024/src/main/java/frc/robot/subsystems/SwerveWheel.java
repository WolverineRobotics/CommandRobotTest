package frc.robot.subsystems;

import java.beans.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;

public class SwerveWheel {

    private CANSparkMax speed_motor;
    private CANSparkMax angle_motor;

    //private PIDController pidController;

    public SwerveWheel(int _speed_motor_port, int _angle_motor_port, PIDController pid){
        this.speed_motor = new CANSparkMax(_speed_motor_port, MotorType.kBrushless);
        this.angle_motor = new CANSparkMax(_angle_motor_port, MotorType.kBrushless);
        //pidController = new PIDController (1, 0, 0);

    }
    
}
