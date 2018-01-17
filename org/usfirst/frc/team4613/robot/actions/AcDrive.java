package org.usfirst.frc.team4613.robot.actions;

import org.usfirst.frc.team4613.robot.OI;
import org.usfirst.frc.team4613.robot.Robot;

import redbacks.arachne.lib.actions.Action;
import redbacks.arachne.lib.checks.ChFalse;
import redbacks.arachne.lib.checks.Check;

public class AcDrive extends Action {

	public AcDrive() {
		super(new ChFalse());
	}
	
	public void onRun(){
		Robot.driver.drivetrain.arcadeDrive(//TODO Fix);
	}

}
