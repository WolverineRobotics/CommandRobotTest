// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Drive.RotateToCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
   
  private DriveSubsystem m_drive = new DriveSubsystem();
  private Command m_driveCommand = new DefaultDriveCommand(m_drive);
  private RotateToCommand m_turnCommand;
  private AutoBalance m_balanceCommand;

  private boolean isTurning = false;
  private boolean isBalancing = false;
  
  //private DriveSubsystem m_drive = new DriveSubsystem();
  //private Command m_driveCommand = new DefaultDriveCommand(m_drive);
  
  
  public DriveSubsystem GetDrive(){ 
    return m_drive;
  }
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OperatorConstants.kDriverControllerPort);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    m_drive = new DriveSubsystem();
    m_driveCommand = new DefaultDriveCommand(m_drive);
    CommandScheduler.getInstance().setDefaultCommand(m_drive, m_driveCommand);
    
  }

  public void TestFaceDirection(double direction){
    if(isTurning){
      return;
    }
    else{
      m_turnCommand = new RotateToCommand(m_drive, direction);
      m_turnCommand.schedule();
    }
    
  }

  public void NoTurnInput(){
    if(isTurning){
      m_turnCommand.EndCall();
      isTurning = false;
    }
  }

  public void Balance(boolean _balance){
    if(isBalancing){
      if(!_balance){
        m_balanceCommand.EndCall();
        isBalancing = false;
      }
      return;
    }
    else if(_balance){
      m_balanceCommand = new AutoBalance(m_drive);
      m_balanceCommand.schedule();
    }
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
