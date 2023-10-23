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
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Encoder;

public class DriveSubsystem extends SubsystemBase {

    // Variables

    // Declaring drive motors
    private CANSparkMax left_1; 
    private CANSparkMax left_2; 
    
    private CANSparkMax right_1; 
    private CANSparkMax right_2;

    private double kP, kI, kD, kIZone, kMaxOutput, kMinOutput, setpoint, errorSum;

    // Declaring motor groups
    private  MotorControllerGroup left_drive;
    private  MotorControllerGroup right_drive;
    
    // Declaring differential drive
    private  DifferentialDrive drive;

    //private SparkMaxPIDController leftBaller;
    //private SparkMaxPIDController rightBaller;

    private DifferentialDriveOdometry m_Odometry;
    private Encoder leftEncoder, rightEncoder;

    private DifferentialDriveKinematics m_Kinematics;

    private double heading;
    private double error;

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

      pigeon.getYaw();
      pigeon.getFusedHeading();

      left_1 = new CANSparkMax(RobotMap.LEFT_MOTOR_1, MotorType.kBrushless);
      left_2 = new CANSparkMax(RobotMap.LEFT_MOTOR_2, MotorType.kBrushless);    
      right_1 = new CANSparkMax(RobotMap.RIGHT_MOTOR_1, MotorType.kBrushless);
      right_2 = new CANSparkMax(RobotMap.RIGHT_MOTOR_2, MotorType.kBrushless);
      
      leftEncoder = new Encoder(Constants.OperatorConstants.kLeftMotorEncoder_A, Constants.OperatorConstants.kLeftMotorEncoder_B);
      rightEncoder = new Encoder(Constants.OperatorConstants.kRightMotorEncoder_A, Constants.OperatorConstants.kRightMotorEncoder_B);

      left_drive = new MotorControllerGroup(left_1, left_2);
      right_drive = new MotorControllerGroup(right_1, right_2);

      drive = new DifferentialDrive(left_drive, right_drive);
    
      left_drive.setInverted(true); /* comment out if robot code doesnt activate */

      left_1.setIdleMode(IdleMode.kBrake);
      left_2.setIdleMode(IdleMode.kBrake);
      right_1.setIdleMode(IdleMode.kBrake);
      right_2.setIdleMode(IdleMode.kBrake);


      // assuming TW is 1.297m (from the 2020 bot) until i find out what it is
      m_Kinematics = new DifferentialDriveKinematics(1.297);

      setDefaultCommand(new DefaultDriveCommand(this));

      /*
       * Get the PID Controllers for both masters (left_1, right_1)
       */
      //SparkMaxPIDController leftBaller =  left_1.getPIDController();
      //SparkMaxPIDController rightBaller = right_1.getPIDController();

      // LEFT ENCODERS
      RelativeEncoder kLeftMotorEncoder_A = left_1.getEncoder(); // 5
      RelativeEncoder kLeftMotorEncoder_B = left_2.getEncoder(); // 6

      // RIGHT ENCODERS
      RelativeEncoder kRightMotorEncoder_A = right_1.getEncoder(); // 7
      RelativeEncoder kRightMotorEncoder_B = right_2.getEncoder(); // 8


      //left_2.follow(left_1, true);
      //right_2.follow(right_1, false);

      // Gain Values
      kP = 0.01;
      kI = 0;
      kD = 0;
      kIZone = 0;
      kMaxOutput = 1;
      kMinOutput = -1;
      
      //leftBaller.setP(kP);
      //rightBaller.setP(kP);
//
      //leftBaller.setI(kI);
      //rightBaller.setI(kI);
//
      //leftBaller.setD(kD);
      //rightBaller.setD(kD);
//
      //leftBaller.setIZone(kIZone);
      //rightBaller.setIZone(kIZone);
 
      /*
       * Ratio between one metre (in inches) to the cycles per inch
       * Comes around to 545.29 encoder ticks per metre.
       */

      kLeftMotorEncoder_A.setPositionConversionFactor(39.37 / RobotMap.CYCLES_PER_INCH);
      kLeftMotorEncoder_B.setPositionConversionFactor(39.37 / RobotMap.CYCLES_PER_INCH);
      kRightMotorEncoder_A.setPositionConversionFactor(39.37 / RobotMap.CYCLES_PER_INCH);
      kRightMotorEncoder_B.setPositionConversionFactor(39.37 / RobotMap.CYCLES_PER_INCH);

      leftEncoder.setDistancePerPulse(1 / 545.29);
      rightEncoder.setDistancePerPulse(1 / 545.29);

      // Since outputs from controllers are -1 -> 1
      //leftBaller.setOutputRange(-1, 1);
      //rightBaller.setOutputRange(-1, 1);

      SmartDashboard.putNumber("kP Gain", kP);
      SmartDashboard.putNumber("kI Gain", kI);
      SmartDashboard.putNumber("kD Gain", kD);
      SmartDashboard.putNumber("I Zone", kIZone);
      SmartDashboard.putNumber("Min Output", kMinOutput);
      SmartDashboard.putNumber("Max Output", kMaxOutput);

      //m_Odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getPigeonHeading()), getLeftEncoderDistance(), getRightEncoderDistance());

  }

  public void motorVoltages(double leftVoltage, double rightVoltage){
    //left_drive.setVoltage(leftVoltage); 
    //right_drive.setVoltage(rightVoltage);
//
    //drive.feed();
  }

  public double getLeftEncoderDistance(){
    return leftEncoder.getDistance();
  }

  public double getRightEncoderDistance(){
    return rightEncoder.getDistance();
  }

  public double getPigeonHeading() {
      double heading = pigeon.getYaw();
      heading %= 360;

      if (heading < 0){
        heading += 360;
      }

      return heading;
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
    error = pigeon.getYaw() - getPigeonHeading();
    drive.arcadeDrive(speed + kP * error, -speed + kP * error);
    // drive.arcadeDrive(speed, 0);
  }

  // moves the robot staight with the given speed
  public void SetDrive(double speed, double rot){
    drive.arcadeDrive(speed, rot);
  }

  public void pidDrive(double distance){
    //leftBaller.setReference(distance, CANSparkMax.ControlType.kPosition);
    //rightBaller.setReference(distance, CANSparkMax.ControlType.kPosition);
  }

  public void turnToAngle(){
    double error = 90 - getPigeonHeading();
    drive.arcadeDrive(kP * error, -kP * error);
  }

  //public double Yaw(){
  //  return pigeon.getYaw();
  //}

  public double Pitch(){
    return pigeon.getPitch();
  }


  @Override
  public void periodic() {
    
    super.periodic();
    //m_Odometry.update(Rotation2d.fromDegrees(getPigeonHeading()), getLeftEncoderDistance(), getRightEncoderDistance());
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
    SmartDashboard.putNumber("Yaw", getPigeonHeading());
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
    //if(p != kP){leftBaller.setP(p); kP = p; rightBaller.setP(p); kP = p;} // When p != kP
    //if(i != kI){leftBaller.setI(i); kI = i; rightBaller.setP(i); kI = i;} // When i != kI
    //if(d != kD){leftBaller.setP(d); kD = d; rightBaller.setP(d); kP = d;} // When d != kD
    //if(iz != kIZone){leftBaller.setP(iz); kIZone = iz; rightBaller.setP(iz); kIZone = iz;} // When iz != kIZone
//
    //if ((min != kMinOutput) || (max != kMaxOutput)){ // When max/min != kMin/KMax
    //  leftBaller.setOutputRange(min, max);
    //  rightBaller.setOutputRange(min, max);
    //  kMinOutput = min; kMaxOutput = max;
    //}

    SmartDashboard.putNumber("Distance", distance);
  }

  //public Pose2d getPose(){
  //  return m_Odometry.getPoseMeters();
  //}
//
  //public DifferentialDriveWheelSpeeds getWheelSpeeds(){
  //  return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  //}
//
  //public void resetOdometry(Pose2d pose){
  //  leftEncoder.reset();
  //  rightEncoder.reset();
  //  m_Odometry.resetPosition(Rotation2d.fromDegrees(getPigeonHeading()), getLeftEncoderDistance(), getRightEncoderDistance(), pose);
  //}

  public void resetPigeonHeading(){
    pigeon.setYaw(0);
    pigeon.setFusedHeading(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
