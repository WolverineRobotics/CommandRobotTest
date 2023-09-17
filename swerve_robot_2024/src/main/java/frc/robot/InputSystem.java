package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class InputSystem {

    private static XboxController dController = new XboxController(0);
    private static XboxController OController = new XboxController(0);

    public static double SpeedLimit(){
        return 1 - (dController.getLeftTriggerAxis() * 0.5);
    } 

    // DRIVER CONTROLS
    public static double DriveSpeed(){
        return dController.getLeftY() * SpeedLimit();
    } 
    public static double DriveRot(){
        return dController.getRightX();
    } 

    public static boolean FaceForward(){
        return dController.getYButton();
    } 
    public static boolean FaceDriver(){
        return dController.getAButton();
    } 
    public static boolean FaceRight(){
        return dController.getBButton();
    } 
    public static boolean FaceLeft(){
        return dController.getXButton();
    } 
    
    public static boolean Balance(){
        return dController.getRightBumper();
    } 

    public static XboxController Driver(){
        return dController;
    } 
}
