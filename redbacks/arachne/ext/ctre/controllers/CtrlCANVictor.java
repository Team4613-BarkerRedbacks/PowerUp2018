package redbacks.arachne.ext.ctre.controllers;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * 
 *
 * @author Sean Zammit
 */
public class CtrlCANVictor extends WPI_VictorSPX
{
	public CtrlCANVictor(int deviceNumber) {
		super(deviceNumber);
	}
}
