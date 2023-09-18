// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.InputSystem;
import frc.robot.Robot;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Encoder;

public class DriveSubsystem extends SubsystemBase {

    // Variables

    // Declaring drive motors
    private final CANSparkMax left_1 = new CANSparkMax(Constants.OperatorConstants.kLeftMotor1, MotorType.kBrushless);
    private final CANSparkMax left_2 = new CANSparkMax(Constants.OperatorConstants.kLeftMotor2, MotorType.kBrushless);
    
    private final CANSparkMax right_1 = new CANSparkMax(Constants.OperatorConstants.kRightMotor1, MotorType.kBrushless);
    private final CANSparkMax right_2 = new CANSparkMax(Constants.OperatorConstants.kRightMotor2, MotorType.kBrushless);

    // Declaring Motor Encoders & Total Distance 
    private final Encoder leftEncoder_1 = new Encoder(5, 6);
    private final Encoder rightEncoder_1 = new Encoder(7, 8);
    
    // Declaring motor groups
    private final MotorControllerGroup left_drive = new MotorControllerGroup(left_1, left_2);
    private final MotorControllerGroup right_drive = new MotorControllerGroup(right_1, right_2);
    
    // Declaring differential drive
    private final DifferentialDrive drive = new DifferentialDrive(left_drive, right_drive);  
    
    // Declaring Accelerometer UwU
    private final Accelerometer accelerometer = new BuiltInAccelerometer();

    // The Pigeon
    private final PigeonIMU pigeon = new PigeonIMU(Constants.OperatorConstants.kPigeonId);
    private double[] position = new double[3];
    private double[] velocity = new double[3];
    //private double proper_yaw = 0;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    pigeon.setYaw(0);

    position[0] = 0;
    position[1] = 0;
    position[2] = 0;

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Updates Robot Position and velocity
    velocity[0] = accelerometer.getX();
    velocity[1] = accelerometer.getY();
    velocity[2] = accelerometer.getZ();

    position[0] += accelerometer.getX();
    position[1] += accelerometer.getY();
    position[2] += accelerometer.getZ();

    SmartDashboard.putNumberArray("Position", position);
    SmartDashboard.putNumberArray("Velocity", velocity);
    SmartDashboard.putNumber("Yaw", pigeon.getYaw());
    SmartDashboard.putNumber("Pitch", pigeon.getPitch());
    SmartDashboard.putNumber("Roll", pigeon.getRoll());

    // Drive Encoder Distance/Ticks
    SmartDashboard.putNumber("DriveEncoderTicks_R", rightEncoder_1.getRaw());
    SmartDashboard.putNumber("DriveEncoderDistance_R", rightEncoder_1.getDistance());
    SmartDashboard.putNumber("DriveEncoderTicks_L", leftEncoder_1.getRaw());
    SmartDashboard.putNumber("DriveEncoderDistance_L", leftEncoder_1.getDistance());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
