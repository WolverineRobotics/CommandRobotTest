// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    //auto = m_robotContainer.getAutonomousCommand();
//
    //if (auto != null) {
    //  auto.schedule();
    //}
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic(){
  
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    /* Facing directions */
    // If the bind is pressed that the user wants to face forward
    if(InputSystem.FaceForward()){
      m_robotContainer.TestFaceDirection(0);
    }
    // If the bind is pressed that the user wants to face backwards
    else if(InputSystem.FaceDriver()){
      m_robotContainer.TestFaceDirection(180);
    }
    
    // If the bind is pressed that the user wants to face left
    else if(InputSystem.FaceLeft()){
      
      m_robotContainer.TestFaceDirection(90);
      // finds wether clockwise or counter clockwise will work
      //if(Math.abs(90 - m_robotContainer.GetDrive().Yaw()) <= Math.abs(-270 - m_robotContainer.GetDrive().Yaw())){
      //  m_robotContainer.TestFaceDirection(90);
      //}
      //else{ m_robotContainer.TestFaceDirection(-270); }
    }
    
    // If the bind is pressed that the user wants to face right
    else if(InputSystem.FaceRight()){
      m_robotContainer.TestFaceDirection(270);
      
      // finds wether clockwise or counter clockwise will work
      //if(Math.abs(270 - m_robotContainer.GetDrive().Yaw()) <= Math.abs(-90 - m_robotContainer.GetDrive().Yaw())){
      //    m_robotContainer.TestFaceDirection(270);
      //   }
      //  else{ m_robotContainer.TestFaceDirection(-90); }
    }
    
    // When none of these requests are pressed
    else{
      m_robotContainer.NoTurnInput();
    }

  
    if(InputSystem.Operator().getStartButtonPressed()){
        m_robotContainer.getPivot().enable();
    }
    if(InputSystem.Operator().getBackButtonPressed()){
        m_robotContainer.getPivot().disable();
    }
    
    if(InputSystem.ToMidCube()){ m_robotContainer.StartMidCubeCommand(); }
    if(InputSystem.ToHighCube()){ m_robotContainer.StartHighCubeCommand(); }
    if(InputSystem.ToIntakePos()){ m_robotContainer.StartIntakePosCommand(); }

    m_robotContainer.Balance(InputSystem.Balance());

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
