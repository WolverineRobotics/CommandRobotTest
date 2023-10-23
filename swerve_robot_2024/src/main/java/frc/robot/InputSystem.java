package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class InputSystem {

    private static XboxController dController = new XboxController(0);
    private static XboxController oController = new XboxController(1);

    public static double SpeedLimit(){ return 1 - (dController.getLeftTriggerAxis() * 0.5); } 
    public static XboxController Driver(){ return dController; } 
    public static XboxController Operator(){ return oController; }

    // DRIVER CONTROLS
    public static double DriveSpeed(){ return dController.getLeftY() * SpeedLimit(); } 
    public static double DriveRot(){ return dController.getRightX(); } 
    public static boolean FaceForward(){ return dController.getYButton(); } 
    public static boolean FaceDriver(){ return dController.getAButton(); } 
    public static boolean FaceRight(){  return dController.getBButton();  } 
    public static boolean FaceLeft(){ return dController.getXButton(); } 
    public static boolean Balance(){ return dController.getRightBumper(); }
    
    // OPERATOR CONTROLS

    public static double ManualElevator(){ return oController.getLeftY(); } 
    public static double ManualPivot(){ return oController.getRightY(); } 
    public static double ManualIntake(){ return ((oController.getLeftTriggerAxis() * 1) + (oController.getRightTriggerAxis() * -1) * 0.5); } 

    public static boolean ToMidCube(){ return oController.getXButtonPressed(); } 
    public static boolean ToHighCube(){ return oController.getYButtonPressed(); } 
    public static boolean ToIntakePos(){ return oController.getAButtonPressed(); } 
    
    public static boolean Retract(){ 
        return
        ( 
            (
                oController.getAButtonReleased() 
                || oController.getYButtonReleased()
                || oController.getXButtonReleased() 
            ) && !(
                oController.getAButton()
                ||oController.getXButton()
                ||oController.getYButton()
                )
        
        ); } 
    
    public static boolean AutoPickup(){ return oController.getLeftBumper(); } 
    public static boolean AutoRetract(){ return oController.getRightBumper(); } 
}
