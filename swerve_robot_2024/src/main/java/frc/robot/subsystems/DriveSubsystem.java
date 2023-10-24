package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.InputSystem;
import frc.robot.RobotMap;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Elevator.DefaultElevatorCommand;

public class DriveSubsystem extends ProfiledPIDSubsystem{

    private CANSparkMax left_1; 
    private CANSparkMax left_2; 
    
    private CANSparkMax right_1; 
    private CANSparkMax right_2;

    private double kP, kI, kD, kIZone, kMaxOutput, kMinOutput;

    private DifferentialDrive drive;

    private MotorControllerGroup left_drive;
    private MotorControllerGroup right_drive;

    private Encoder leftEncoder, rightEncoder;

    private double heading;
    private double error;

    private final PigeonIMU pigeon = new PigeonIMU(2);
    private double[] position = new double[3];
    private double[] velocity = new double[3];

    public DriveSubsystem(){

        super(
            new ProfiledPIDController(0.01, 0, 0.0,
            new TrapezoidProfile.Constraints(
                OperatorConstants.kMaxDriveVelocity,
                OperatorConstants.kMaxDriveAcceleration)));

        
        // Set yaw!!!!!!!
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

        left_drive.setInverted(true); /* comment out if robot code doesnt activate */

      
        leftEncoder = new Encoder(Constants.OperatorConstants.kLeftMotorEncoder_A, Constants.OperatorConstants.kLeftMotorEncoder_B);                       
        rightEncoder = new Encoder(Constants.OperatorConstants.kRightMotorEncoder_A, Constants.OperatorConstants.kRightMotorEncoder_B);
        
        leftEncoder.setDistancePerPulse(1 / 545.29);
        rightEncoder.setDistancePerPulse(1 / 545.29);

        left_1.setIdleMode(IdleMode.kBrake);
        left_2.setIdleMode(IdleMode.kBrake);
        right_1.setIdleMode(IdleMode.kBrake);
        right_2.setIdleMode(IdleMode.kBrake);

        setDefaultCommand(new DefaultDriveCommand(this));

    }

    @Override 
    public void periodic(){

        super.periodic();

        SmartDashboard.putNumber("Yaw", getPigeonHeading());
        SmartDashboard.putNumber("Pitch", pigeon.getPitch());
        SmartDashboard.putNumber("Roll", pigeon.getRoll());

       // SmartDashboard.putNumber("Distance", distance);
    }

    public void ManualControl(){
        
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
  public void MoveStraightPropotionate(double speed, double target){
    error =  target - getPigeonHeading();
    drive.arcadeDrive(speed + kP * error, -speed + kP * error);
    // drive.arcadeDrive(speed, 0);
  }
  public void MoveStraight(double speed){
    drive.arcadeDrive(speed, 0);
    // drive.arcadeDrive(speed, 0);
  }

  // moves the robot staight with the given speed
  public void SetDrive(double speed, double rot){
    drive.arcadeDrive(speed, rot);
  }

  public void turnToAngle(){
    double error = 90 - getPigeonHeading();
    drive.arcadeDrive(kP * error, -kP * error);
  }

    @Override
    protected void useOutput(double output, edu.wpi.first.math.trajectory.TrapezoidProfile.State setpoint) {
        // TODO Auto-generated method stub
        
    }

    public double Pitch(){
      return pigeon.getPitch();
    }
    
    @Override
    protected double getMeasurement() {
        return getPigeonHeading();
    }

    public double getPigeonHeading() {
        double heading = pigeon.getYaw();
        heading %= 360;

        if (heading < 0){
        heading += 360;
        }

        return heading;
    }
    
}