// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Drive.RotateToCommand;
import frc.robot.commands.OperatorSetPoints.IntakePositionCommand;
import frc.robot.commands.OperatorSetPoints.MidCubeCommand;
import frc.robot.commands.OperatorSetPoints.TopGridCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  //private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
   
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final PivotSubsystem m_pivot = new PivotSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private Command m_driveCommand;
  private RotateToCommand m_turnCommand;
  private AutoBalance m_balanceCommand;

  private boolean isTurning = false;
  private boolean isBalancing = false;
  
  private double side = 0;//SmartDashboard.getNumber("Side", 0); // 0 for blue, 1 for red
  private double start_pos= 0;//SmartDashboard.getNumber("StartingPos", 0); // 0: leftmost, 1: center, 2: rightmost

  //private AutoCommand autoWeBall = new AutoCommand(weBalling);
  
  public DriveSubsystem GetDrive(){ 
    return m_drive;
  }
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   // CommandScheduler.getInstance().setDefaultCommand(m_drive, new DefaultDriveCommand(m_drive));    
  }

  public void UpdateConfig(){
    //side = SmartDashboard.getNumber("Side", 0); // 0 for blue, 1 for red
    //start_pos= SmartDashboard.getNumber("StartingPos", 0); // 0: leftmost, 1: center, 2: rightmost
  }

  public void StartMidCubeCommand(){ new MidCubeCommand(m_pivot, m_elevator).schedule(); }
  public void StartHighCubeCommand(){ new TopGridCommand(m_pivot, m_elevator).schedule(); }
  public void StartIntakePosCommand(){ new IntakePositionCommand(m_pivot, m_elevator).schedule(); }

  public void TestFaceDirection(double direction){
    if(isTurning){
      return;
    }
    else{
      m_turnCommand = new RotateToCommand(m_drive, direction);
      m_turnCommand.schedule();
    }
    
  }

  public PivotSubsystem getPivot(){
    return m_pivot;
  }

  public void NoTurnInput(){
    if(isTurning){
      m_turnCommand.EndCall();
      m_drive.Rotate(0);
      //m_turnCommand.cancel();
      isTurning = false;
    }
  }

  public void Balance(boolean _balance){
    if(isBalancing){
      if(!_balance){
        //m_balanceCommand.EndCall();
        m_balanceCommand.cancel();
        isBalancing = false;
      }
      return;
    }
    else if(_balance){
      m_balanceCommand = new AutoBalance(m_drive);
      m_balanceCommand.schedule();
    }
  }

  //public Command getAutonomousCommand() {
  //  // WE BALL
  //  return autoWeBall;
  //}
}
