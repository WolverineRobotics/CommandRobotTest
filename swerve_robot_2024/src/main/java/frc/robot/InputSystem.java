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
        return dController.getYButtonPressed();
    } 
    public static boolean FaceDriver(){
        return dController.getAButtonPressed();
    } 
    public static boolean FaceRight(){
        return dController.getBButtonPressed();
    } 
    public static boolean FaceLeft(){
        return dController.getXButtonPressed();
    } 

    public static XboxController Driver(){
        return dController;
    } 
}
