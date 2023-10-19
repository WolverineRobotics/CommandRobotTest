package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class InputSystem {

    private static XboxController dController = new XboxController(0);
    private static XboxController oController = new XboxController(1);

    public static double SpeedLimit(){
        return 1 - (dController.getLeftY() * 0.5); 
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
    
    // OPERATOR CONTROLS
    public static XboxController Operator(){
        return oController;
    } 
    public static double ManualElevator(){
        return oController.getLeftY();
    } 
    public static double ManualPivot(){
        return oController.getLeftY();
    } 
    public static double ManualIntake(){
        return (oController.getLeftTriggerAxis()) + (oController.getRightTriggerAxis() * -1) * 0.5;
    } 
    
    
    public static boolean ToMidCube(){
        return oController.getXButtonPressed();
    } 
}
