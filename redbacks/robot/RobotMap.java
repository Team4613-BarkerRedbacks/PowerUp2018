package redbacks.robot;

import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;
import redbacks.arachne.ext.ctre.controllers.CtrlCANVictor;

public class RobotMap
{
	public static final double armSpeed = 1;
	
	private static final CtrlCANTalon
		talon2	= new CtrlCANTalon(2),
		talon3	= new CtrlCANTalon(3),
		talon4	= new CtrlCANTalon(4),
		talon5	= new CtrlCANTalon(5),
		talon6	= new CtrlCANTalon(6),
		talon7  = new CtrlCANTalon(7),
		talon8	= new CtrlCANTalon(8);
	
	private static final CtrlCANVictor
		victor9		= new CtrlCANVictor(9),
		victor10	= new CtrlCANVictor(10);
		
	public static final CtrlCANTalon
		idMotDriveR1 = talon2,
		idMotDriveL1 = talon3,
		idMotDriveR2 = talon4,
		idMotDriveL2 = talon5,
		idMotDriveR3 = talon6,
		idMotDriveL3 = talon7,
		idMotArm	 = talon8;
	
	public static final CtrlCANVictor
		idMotIntakeL = victor9,
		idMotIntakeR = victor10;
		
	public static final int
		idSolCentreEncoder = 0,
		idSolRightIntake = 1,
		idSolLeftIntake = 2,
		idSolShooter = 3,
		idSolLock = 4;

	public static final double
	//P = 0.00008D
		drivePIDMotorkP = 0.00007D, drivePIDMotorkI = 0.0000001D, drivePIDMotorkD = 0.00001,
		drivePIDGyrokP = 0.03D, drivePIDGyrokI = 0.0001, drivePIDGyrokD = 0.002;

}