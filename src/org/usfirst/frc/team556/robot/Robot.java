
package org.usfirst.frc.team556.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team556.robot.subsystems.DriveTrain;
import org.usfirst.frc.team556.robot.subsystems.PDP;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	
	public static DriveTrain drivetrain;
	public static OI oi;
	public static PDP PowerDistPanel;
	public int AutoProgramNumber;
	
    
	//public static SendableChooser AutoChooser; 

    Command autonomousCommand;
    
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
		
		drivetrain = new DriveTrain();
		PowerDistPanel = new PDP();
		oi = new OI();
		SmartDashboard.putData(drivetrain);
        SmartDashboard.putNumber("Hello World", 2);
    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

    public void autonomousInit() {
    	if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        log();
        
    }

    public void teleopInit() {
    	
		// This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove this line or comment it out.
        
    	Robot.drivetrain.gyro.reset();
    	if (autonomousCommand != null) autonomousCommand.cancel();
        
        //while (isEnabled()){
        	
       // }
        
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        	Scheduler.getInstance().run();
        	
        log();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
    
    private void log() {
    	drivetrain.log();
    	PowerDistPanel.log();
    }
}
