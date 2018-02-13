package redbacks.arachne.lib.override;

import redbacks.arachne.ext.motion.MotionSettings;

/**
 * 
 *
 * @author Sean Zammit
 */
public class MotionSettings2 extends MotionSettings
{
	public static final double
		drivePIDMotorkP = 2.0E-5, drivePIDMotorkI = 1.0E-8, drivePIDMotorkD = 7.0E-6,
		drivePIDGyrokP = 4.0E-2, drivePIDGyrokI = 1.0E-5, drivePIDGyrokD = 1.0E-2;
}
