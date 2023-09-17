// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.InputSystem;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {

    // Variables

    // Declaring drive motors
    private final CANSparkMax left_1 = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax left_2 = new CANSparkMax(0, MotorType.kBrushless);
    
    private final CANSparkMax right_1 = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax right_2 = new CANSparkMax(0, MotorType.kBrushless);
    
    // Declaring motor groups
    private final MotorControllerGroup left_drive = new MotorControllerGroup(left_1, left_2);
    private final MotorControllerGroup right_drive = new MotorControllerGroup(right_1, right_2);
    
    // Declaring differential drive
    private final DifferentialDrive drive = new DifferentialDrive(left_drive, right_drive);  

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase DriveMethodCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drive.arcadeDrive(InputSystem.DriveSpeed(), InputSystem.DriveRot());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
