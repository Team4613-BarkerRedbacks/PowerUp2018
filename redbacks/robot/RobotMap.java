package redbacks.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import redbacks.arachne.ext.ctre.controllers.CtrlCANTalon;
import redbacks.arachne.ext.ctre.controllers.CtrlCANVictor;

public class RobotMap
{
	public static final double
		armMaxSpeed = 1,
		armKP = 1.8E-3, armKI = 0, armKD = 4E-4,
		intakeSpeed = 0.7, intakeSlowSpeed = 0.4, intakeFastSpeed = 1,
		stoppedMoveThreshold = 0.2, stoppedTurnThreshold = 10,
		cubeTrackKP = 0.1;
	
	public static final int
		armSlightPos = 75, armScalePos = 150, armSwitchPos = 350, armBasePos = 1025,
		sideEncoderTicksPerMetre = 13800;
	
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
		victor10	= new CtrlCANVictor(10),
		victor11	= new CtrlCANVictor(11),
		victor12	= new CtrlCANVictor(12);
		
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
		idMotIntakeR = victor10,
		idMotClimb1 = victor11,
		idMotClimb2 = victor12;
		
	public static final int
		idSolCentreEncoder = 2,
		idSolShooter2 = 7,
		idSolShooterHigh = 1,
		idSolRightIntake = 4,
		idSolLeftIntake = 5,
		idSolShooter1 = 0,
		idSolLock = 6,
		idSolClimbRelease = 3;
	
	public static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
}