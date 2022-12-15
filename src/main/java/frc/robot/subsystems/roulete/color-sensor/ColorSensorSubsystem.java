// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.colorSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.hamosad1657.lib.sensors.HaColorSensor;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/Java/Read%20RGB%20Values/src/main/java/frc/robot/Robot.java

public class ColorSensorSubsystem extends SubsystemBase {
	/** Creates a new ColorSensorSubsystem. */
	private static ColorSensorSubsystem instance;

	public static ColorSensorSubsystem getInstance() {
		if (instance == null) {
			instance = new ColorSensorSubsystem();
		}
		return instance;
	}

	private final HaColorSensor colorSensor;
	private final I2C.Port i2cPort;
	// private final ColorSensorV3 m_colorSensor;
	// private Color detectedColor;
	private final ShuffleboardTab tab;
	// private final NetworkTableEntry red, blue, green, proximity;
	// private double proximityVal;

	private ColorSensorSubsystem() {
		this.tab = Shuffleboard.getTab("ColorSensor");
		i2cPort = I2C.Port.kMXP;
		this.colorSensor = new HaColorSensor(this.i2cPort, this.tab);
		// m_colorSensor = new ColorSensorV3(i2cPort);
		// this.blue = this.tab.add("Blue", 0.0).getEntry();
		// this.red = this.tab.add("Red", 0.0).getEntry();
		// this.green = this.tab.add("Green", 0.0).getEntry();
		// this.proximity = this.tab.add("Proximity", 0.0).getEntry();
	}

	public RawColor getColor() {
		return this.colorSensor.getColor();
	}

	public double getRed() {
		return this.colorSensor.getRed();
	}

	public double getBlue() {
		return this.colorSensor.getBlue();
	}

	public double getGreen() {
		return this.colorSensor.getGreen();
	}

	public boolean checkForBlue() {
		return this.colorSensor.isColorInRange(ColorSensorConstants.minBlue, ColorSensorConstants.maxBlue);
	}

	public double getProximity() {
		return this.colorSensor.getProximity();
	}
	// public Color getColor() {
	// return detectedColor;
	// }

	// public double getRed() {
	// return detectedColor.red;
	// }

	// public double getBlue() {
	// return detectedColor.blue;
	// }

	// public double getGreen() {
	// return detectedColor.green;
	// }

	// private void setProximity(double gottenProximity) {
	// this.proximityVal = gottenProximity;
	// }

	// public double getProximity() {
	// return proximityVal;
	// }

	// private boolean checkRedValues() {
	// return detectedColor.red <= ColorSensorConstants.kMaxRedTarget
	// && detectedColor.red >= ColorSensorConstants.kMinRedTarget;
	// }

	// private boolean checkGreenValues() {
	// return detectedColor.green <= ColorSensorConstants.kMaxGreenTarget
	// && detectedColor.green >= ColorSensorConstants.kMinGreenTarget;
	// }

	// private boolean checkBlueValues() {
	// return detectedColor.blue <= ColorSensorConstants.kMaxBlueTarget
	// && detectedColor.blue >= ColorSensorConstants.kMinBlueTarget;
	// }

	// public boolean checkForBlue() {
	// if (checkRedValues() && checkBlueValues() && checkGreenValues()) // change to
	// the real checking algorithem
	// return true;
	// return false;
	// }

	@Override
	public void periodic() {
		// detectedColor = m_colorSensor.getColor();
		// setProximity(m_colorSensor.getProximity());
		// this.blue.setDouble(getBlue());
		// this.red.setDouble(getRed());
		// this.green.setDouble(getGreen());
		// this.proximity.setDouble(getProximity());
	}
}
