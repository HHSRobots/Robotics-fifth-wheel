package org.usfirst.frc.team556.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.AnalogGyro;

import org.usfirst.frc.team556.robot.commands.DriveWithJoystick;
//import edu.wpi.first.wpilibj.Timer;

public class DriveTrain extends Subsystem {
    private SpeedController left_wheels, right_wheels, fifth_wheel;
    private RobotDrive drive;
    public AnalogGyro gyro;
    private Boolean wasHeld;
    
    
    public DriveTrain(){
    	super();
    	left_wheels = new Victor(0);
    	right_wheels = new Victor(1);
    	fifth_wheel = new Victor(2);
    	drive = new RobotDrive(left_wheels, right_wheels);
    	gyro = new AnalogGyro(0);
    	wasHeld = false;
    }
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
    	setDefaultCommand(new DriveWithJoystick());
    }
    
    public void drivemanual(double left, double right){
    	drive.tankDrive(left,right);
    }
    
    public void drive(Joystick joystick_driver){
    	double z = 0; // amount of rotation applied to robot -128 through 128, 0 is none
    	double Kp = 0.025; // constant that gives magnitude of rotation correction (recomended is 0.03)
    	
    	if (joystick_driver.getRawButton(12) == true ){	
    		z =-joystick_driver.getZ();
    		wasHeld = true;
    	} else if (wasHeld && (Math.abs(gyro.getRate()) <= 1) ){
    		gyro.reset();
    		wasHeld = false;
    		double angle = gyro.getAngle();
    		z = Kp * angle;
    	} else if(wasHeld == false){
    	// inset gyro balancing code here to compensate for skew
    		double angle = gyro.getAngle();
    		z = Kp * angle;
    	}
    	
    	drive.arcadeDrive(joystick_driver.getY(),z);
    	fifth_wheel.set(-joystick_driver.getX());
    }
    
    
    public void log() {
    	SmartDashboard.putNumber("Fork spiun :",gyro.getRate());
    }
}

