// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.InputSystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Drive.DefaultDriveCommand;
import pabeles.concurrency.IntOperatorTask.Min;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Encoder;

public class DriveSubsystem extends SubsystemBase {

    // Variables

    // Declaring drive motors
    private CANSparkMax left_1; 
    private CANSparkMax left_2; 
    
    private CANSparkMax right_1; 
    private CANSparkMax right_2;

    private double kP, kI, kD, kIZone, kMaxOutput, kMinInput, setpoint, errorSum;

    // declaring Motor Encoders & Total Distance 
    // private final Encoder leftEncoder_1 = new Encoder(5, 6);
    // private final Encoder rightEncoder_1 = new Encoder(7, 8);
    
    // Declaring motor groups
    private  MotorControllerGroup left_drive;
    private  MotorControllerGroup right_drive;
    
    // Declaring differential drive
    private  DifferentialDrive drive;  

    private SparkMaxPIDController leftBaller;
    private SparkMaxPIDController rightBaller;
    
    // Declaring Accelerometer UwU
    //private final Accelerometer accelerometer = new BuiltInAccelerometer();

    // The Pigeon
    private final PigeonIMU pigeon = new PigeonIMU(2);
    private double[] position = new double[3];
    private double[] velocity = new double[3];

    //private double proper_yaw = 0;
    
    /** Creates a new ExampleSubsystem. */
    public DriveSubsystem() {
      pigeon.setYaw(0);
      
      position[0] = 0;
      position[1] = 0;
      position[2] = 0;

      left_1 = new CANSparkMax(RobotMap.LEFT_MOTOR_1, MotorType.kBrushless);
      left_2 = new CANSparkMax(RobotMap.LEFT_MOTOR_2, MotorType.kBrushless);    
      right_1 = new CANSparkMax(RobotMap.RIGHT_MOTOR_1, MotorType.kBrushless);
      right_2 = new CANSparkMax(RobotMap.RIGHT_MOTOR_2, MotorType.kBrushless);
  
      left_drive = new MotorControllerGroup(left_1, left_2);
      right_drive = new MotorControllerGroup(right_1, right_2);

      drive = new DifferentialDrive(left_drive, right_drive);
    
      left_drive.setInverted(true);


      left_1.setIdleMode(IdleMode.kBrake);
      left_2.setIdleMode(IdleMode.kBrake);
      right_1.setIdleMode(IdleMode.kBrake);
      right_2.setIdleMode(IdleMode.kBrake);

      setDefaultCommand(new DefaultDriveCommand(this));

      /*
       * Get the PID Controllers for both masters (left_1, right_1)
       */
      SparkMaxPIDController leftBaller =  left_1.getPIDController();
      SparkMaxPIDController rightBaller = right_1.getPIDController();

      // LEFT ENCODERS
      RelativeEncoder kLeftMotorEncoder_A = left_1.getEncoder(); 
      RelativeEncoder kLeftMotorEncoder_B = left_2.getEncoder(); 

      // RIGHT ENCODERS
      RelativeEncoder kRightMotorEncoder_A = right_1.getEncoder();
      RelativeEncoder kRightMotorEncoder_B = right_2.getEncoder();
      
      left_2.follow(left_1, true);
      right_2.follow(right_1, true);

      // Gain Values
      kP = 0.1;
      kI = 0.01;
      kD = 1.3;
      kIZone = 0;
      kMaxOutput = 1;
      kMinInput = -1;
      
      leftBaller.setP(kP);
      rightBaller.setP(kP);

      leftBaller.setI(kI);
      rightBaller.setI(kI);

      leftBaller.setD(kD);
      rightBaller.setD(kD);

      leftBaller.setIZone(kIZone);
      rightBaller.setIZone(kIZone);
 
      /*
       * Ratio between one metre (in inches) to the cycles per inch
       * Comes around to 545.29 encoder ticks per metre.
       */
      kLeftMotorEncoder_A.setPositionConversionFactor(39.37 / RobotMap.CYCLES_PER_INCH);
      kLeftMotorEncoder_B.setPositionConversionFactor(39.37 / RobotMap.CYCLES_PER_INCH);
      kRightMotorEncoder_A.setPositionConversionFactor(39.37 / RobotMap.CYCLES_PER_INCH);
      kRightMotorEncoder_B.setPositionConversionFactor(39.37 / RobotMap.CYCLES_PER_INCH);

      // Since outputs from controllers are -1 -> 1

      leftBaller.setOutputRange(-1, 1);
      rightBaller.setOutputRange(-1, 1);

      SmartDashboard.putNumber("kP Gain", kP);
      SmartDashboard.putNumber("kI Gain", kI);
      SmartDashboard.putNumber("kD Gain", kD);
      SmartDashboard.putNumber("I Zone", kIZone);
      SmartDashboard.putNumber("Min Output", kMinInput);
      SmartDashboard.putNumber("Max Output", kMaxOutput);

    //proper_yaw = pigeon.getYaw();
  }

  public CommandBase DriveMethodCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  // Drive Methods

  // Manual default driving mode
  public void ArcadeDrive(){
    drive.arcadeDrive(InputSystem.DriveSpeed(), InputSystem.DriveRot());
  }

  // Rotates the robot at the speed given
  public void Rotate(double speed){
    drive.arcadeDrive(0, speed);
  }
  
  // moves the robot staight with the given speed
  public void MoveStraight(double speed){
    drive.arcadeDrive(speed, 0);
  }

  // moves the robot staight with the given speed
  public void SetDrive(double speed, double rot){
    drive.arcadeDrive(speed, rot);
  }

  public double Yaw(){
    return pigeon.getYaw();
  }

  public double Pitch(){
    return pigeon.getPitch();
  }

  public void pidDrive(double distance){
    leftBaller.setReference(distance, CANSparkMax.ControlType.kPosition);
    rightBaller.setReference(distance, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Updates Robot Position and velocity
    //velocity[0] = accelerometer.getX();
    //velocity[1] = accelerometer.getY();
    //velocity[2] = accelerometer.getZ();
//
    //position[0] += accelerometer.getX();
    //position[1] += accelerometer.getY();
    //position[2] += accelerometer.getZ();

    //SmartDashboard.putNumberArray("Position", position);
    //SmartDashboard.putNumberArray("Velocity", velocity);
    SmartDashboard.putNumber("Yaw", pigeon.getYaw());
    SmartDashboard.putNumber("Pitch", pigeon.getPitch());
    SmartDashboard.putNumber("Roll", pigeon.getRoll());
    
    double p = SmartDashboard.getNumber("kP Gain", 0);
    double i = SmartDashboard.getNumber("kI Gain", 0);
    double d = SmartDashboard.getNumber("kD Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double min = SmartDashboard.getNumber("Min Ouput", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double distance = SmartDashboard.getNumber("Distance", 0);
    
    /*
     * This will update any values that have changed on the PID controllers
     */
    if(p != kP){leftBaller.setP(p); kP = p; rightBaller.setP(p); kP = p;}
    if(i != kI){leftBaller.setI(i); kI = i; rightBaller.setP(i); kI = i;}
    if(d != kD){leftBaller.setP(d); kD = d; rightBaller.setP(d); kP = d;}
    if(iz != kIZone){leftBaller.setP(iz); kIZone = iz; rightBaller.setP(iz); kIZone = iz;}
    if ((min != kMinInput) || (max != kMaxOutput)){
      leftBaller.setOutputRange(min, max);
      rightBaller.setOutputRange(min, max);
      kMinInput = min; kMaxOutput = max;
    }

    SmartDashboard.putNumber("Distance", distance);
    
  }

  /* 
  public double getLeftEncoderA_Distance(Encoder kLeftMotorEncoder_A){
    return kLeftMotorEncoder_A.getDistance();
  }

  public double getLeftEncoderB_Distance(Encoder kLeftMotorEncoder_B){
    return kLeftMotorEncoder_B.getDistance();
  }

  public double getRightEncoderA_Distance(Encoder kRightMotorEncoder_A){
    return kRightMotorEncoder_A.getDistance();
  }

  public double getRightEncoderB_Distance(Encoder kRightMotorEncoder_B){
    return kRightMotorEncoder_B.getDistance();
  }
*/

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
