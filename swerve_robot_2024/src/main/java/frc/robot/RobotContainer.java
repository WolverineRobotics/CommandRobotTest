// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.DefaultDriveCommand;
import frc.robot.commands.Drive.ForwardDriveCommand;
import frc.robot.commands.Drive.RotateToCommand;
import frc.robot.commands.Intake.RunIntakeCommand;
import frc.robot.commands.OperatorSetPoints.IntakePositionCommand;
import frc.robot.commands.OperatorSetPoints.LowDropCommand;
import frc.robot.commands.OperatorSetPoints.MidCubeCommand;
import frc.robot.commands.OperatorSetPoints.RetractPositionCommand;
import frc.robot.commands.OperatorSetPoints.TopGridCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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


  /* Auto Commands
   * UNFINISHED, WILL BE POLISHED LATER, JUST A CONCEPT
   */
  //private AutoCommand autoWeBall = new AutoCommand(weBalling);

  public Command getAutonomousCommand() {
    // ----------------------------------------IMPORTANT---------------------------------
    // THIS IS WHERE YOU DETERMING THE COMMAND YOU WANT TO DO IN AUTO. RETURN NULL FOR NONE
    SequentialCommandGroup selected_commandGroup = test_auto;
    return selected_commandGroup;
  }

  // Put anything here to test
  public SequentialCommandGroup test_auto = new SequentialCommandGroup(
    new ForwardDriveCommand(m_drive, 0.4, 1000), 
    new ForwardDriveCommand(m_drive, -0.2, 1000),
    new AutoBalance(m_drive)
    );
    
  // ----------------------- DEFINED AUTOS TO USE IN getAutonomousCommand() ------------------------------
  public SequentialCommandGroup Top_balance_auto = new SequentialCommandGroup(
    new TopGridCommand(m_pivot, m_elevator),
    new RunIntakeCommand(m_intake, 0.75),
    new RetractPositionCommand(m_pivot, m_elevator),
    new ForwardDriveCommand(m_drive, 0.6, 3000),
    new ForwardDriveCommand(m_drive, 0, 500),
    new ForwardDriveCommand(m_drive, -0.75, 1000),
    new AutoBalance(m_drive)
  );
  public SequentialCommandGroup mid_balance_auto = new SequentialCommandGroup(
    new MidCubeCommand(m_pivot, m_elevator),
    new RunIntakeCommand(m_intake, 0.75),
    new RetractPositionCommand(m_pivot, m_elevator),
    new ForwardDriveCommand(m_drive, 0.6, 3000),
    new ForwardDriveCommand(m_drive, 0, 500),
    new ForwardDriveCommand(m_drive, -0.75, 1000),
    new AutoBalance(m_drive)
  );
  public SequentialCommandGroup side_top_mobility_auto = new SequentialCommandGroup(
    new TopGridCommand(m_pivot, m_elevator),
    new RunIntakeCommand(m_intake, 0.75),
    new RetractPositionCommand(m_pivot, m_elevator),
    new ForwardDriveCommand(m_drive, 0.5, 2500)
  );
  public SequentialCommandGroup side_mid_mobility_auto = new SequentialCommandGroup(
    new MidCubeCommand(m_pivot, m_elevator),
    new RunIntakeCommand(m_intake, 0.75),
    new RetractPositionCommand(m_pivot, m_elevator),
    new ForwardDriveCommand(m_drive, 0.5, 2500)
  );
  public SequentialCommandGroup side_top_auto = new SequentialCommandGroup(
    new TopGridCommand(m_pivot, m_elevator),
    new RunIntakeCommand(m_intake, 0.75),
    new RetractPositionCommand(m_pivot, m_elevator)
  );
  public SequentialCommandGroup side_mid_auto = new SequentialCommandGroup(
    new MidCubeCommand(m_pivot, m_elevator),
    new RunIntakeCommand(m_intake, 0.75),
    new RetractPositionCommand(m_pivot, m_elevator)
  );
  
  public DriveSubsystem GetDrive(){ 
    return m_drive;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  }

  public void UpdateConfig(){
  }

  // Methods for starting new commands once called
  public void StartMidCubeCommand(){ new MidCubeCommand(m_pivot, m_elevator).schedule(); }
  public void StartHighCubeCommand(){ new TopGridCommand(m_pivot, m_elevator).schedule(); }
  public void StartIntakePosCommand(){ new IntakePositionCommand(m_pivot, m_elevator).schedule(); }
  public void StartLowDropCommand(){ new LowDropCommand(m_pivot, m_elevator).schedule(); }
  public void StartRetractCommand(){ new RetractPositionCommand(m_pivot, m_elevator).schedule(); }
  
  // Starts new rotate to command once turning is false
  public void TestFaceDirection(double direction){
    if(isTurning){
      return;
    }
    else{
      m_turnCommand = new RotateToCommand(m_drive, direction);
      m_turnCommand.schedule();
    }
    
  }
  
  // Accessing subsystems
  public PivotSubsystem getPivot(){
    return m_pivot;
  }
  public ElevatorSubsystem getElevator(){
    return m_elevator;
  }

  public void NoTurnInput(){
    //if(isTurning){
    //  m_turnCommand.cancel();
    //  m_turnCommand.EndCall();
    //  m_drive.Rotate(0);
    //  m_turnCommand.cancel();
    //  isTurning = false;
    //}
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
}
