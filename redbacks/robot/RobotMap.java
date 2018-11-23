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
		armSlightPosTele = 75, armSlightPos = 100, armScalePos = 150, armSwitchPos = 350, armBasePos = 1025,
		sideEncoderTicksPerMetre = 14100;
	
	private static final CtrlCANTalon
		talon2	= new CtrlCANTalon(2);
	
	private static final CtrlCANVictor
		victor3		= new CtrlCANVictor(3),
		victor8		= new CtrlCANVictor(8),
		victor9		= new CtrlCANVictor(9),
		victor10	= new CtrlCANVictor(10),
		victor11	= new CtrlCANVictor(11);
		
	public static final CtrlCANTalon
		idMotDriveR = talon2;
	
	public static final CtrlCANVictor
		idMotDriveL = victor3,
		idMotArm = victor8,
		idMotIntakeL = victor9,
		idMotIntakeR = victor10,
		idMotClimb = victor11;
		
	public static final int
		idSolShooter = 0,
		idSolClimbRelease1 = 1,
		idSolClimbRelease2 = 2,
		idSolRightIntake = 4,
		idSolLeftIntake = 5,
		idSolLock = 6;
	
	public static final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
}