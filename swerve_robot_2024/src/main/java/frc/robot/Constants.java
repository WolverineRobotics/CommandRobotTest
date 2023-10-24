// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

    /* Controller ports */
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
    
    /* CAN ID constants */
    public static final int kPigeonId = 0;
    
    public static final int kLeftMotor1 = 1;
    public static final int kLeftMotor2 = 2; 
    
    public static final int kRightMotor1 = 3;
    public static final int kRightMotor2 = 4;

    public static final int kLeftMotorEncoder_A = 5; 
    public static final int kLeftMotorEncoder_B = 6;

    public static final int kRightMotorEncoder_A = 7; 
    public static final int kRightMotorEncoder_B = 8;

    public static final int kElevatorMotor1 = 14; 
    public static final int kElevatorMotor2 = 10;
    
    public static final int kPivotMotor = 15;
    public static final int kIntakeMotor = 16;


    /* Trapezoidal constraints */
    public static final int kMaxElevatorVelocity = 125;
    public static final int kMaxPivotVelocity = 160;

    public static final int kMaxElevatorAcceleration = 300;
    public static final int kMaxPivotAcceleration = 300;

    public static final int kMaxDriveVelocity = 300;
    public static final int kMaxDriveAcceleration = 300;
  }
}
