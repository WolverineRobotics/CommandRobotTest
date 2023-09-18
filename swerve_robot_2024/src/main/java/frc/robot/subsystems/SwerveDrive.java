package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.InputSystem;

public class SwerveDrive {
    private CANSparkMax top_left_wheel;
    private CANSparkMax top_left_rotater;
    
    private CANSparkMax top_right_wheel;
    private CANSparkMax top_right_rotater;

    private CANSparkMax bottom_left_wheel;
    private CANSparkMax bottom_left_rotater;
    
    private CANSparkMax bottom_right_wheel;
    private CANSparkMax bottom_right_rotater;

    private final double length_of_robot = 32;
    private final double width_of_robot = 28;

    public void arcadeDrive(){
        
        double speed_x = InputSystem.Driver().getLeftX();  
        double speed_y = InputSystem.Driver().getLeftY();        
        double rot = InputSystem.Driver().getRightX();  

        double hyp = Math.sqrt( (length_of_robot * length_of_robot) + (width_of_robot * width_of_robot) );
        
        //why? idk
        speed_y *= -1;

        //sin and cosine of robot length and width
        double sin = (length_of_robot / hyp);
        double cos = (width_of_robot / hyp); 

        // values for wheel sides
        double bottom_x = speed_x - rot * sin;
        double top_x = speed_x + rot * sin;
        double left_y = speed_y - rot * cos;
        double right_y = speed_y + rot * cos;

        // Speeds for 4 wheels
        double top_left_speed = Math.sqrt( (top_x*top_x) + (left_y*left_y) );
        double top_right_speed = Math.sqrt( (top_x*top_x) + (right_y*right_y) );
        double bottom_left_speed = Math.sqrt( (bottom_x*bottom_x) + (left_y*left_y) );
        double bottom_right_speed = Math.sqrt( (bottom_x*bottom_x) + (right_y*right_y) );
        
        // Angles for 4 wheels
        double top_left_angle = Math.atan2(top_x, left_y) / Math.PI;
        double top_right_angle = Math.atan2(top_x, right_y) / Math.PI;
        double bottom_left_angle = Math.atan2(bottom_x, left_y) / Math.PI;
        double bottom_right_angle = Math.atan2(bottom_x, right_y) / Math.PI;
        
        
    }
}
