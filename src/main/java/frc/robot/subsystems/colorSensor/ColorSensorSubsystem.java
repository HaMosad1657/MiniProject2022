// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.colorSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
//import frc.robot.subsystems.colorSensor.ColorSensorConstants;

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

	private final I2C.Port i2cPort;
	private final ColorSensorV3 m_colorSensor;
	private Color detectedColor;
	private final ShuffleboardTab tab;
	private final NetworkTableEntry red, blue, green;

	private ColorSensorSubsystem() {
		this.tab = Shuffleboard.getTab("ColorSensor");
		i2cPort = I2C.Port.kMXP;
		m_colorSensor = new ColorSensorV3(i2cPort);
		this.blue = this.tab.add("Blue", 0.0).getEntry();
		this.red = this.tab.add("Red", 0.0).getEntry();
		this.green = this.tab.add("Green", 0.0).getEntry();
	}

	public Color getColor() {
		return detectedColor;
	}

	public double getRed() {
		return detectedColor.red;
	}

	public double getBlue() {
		return detectedColor.blue;
	}

	public double getGreen() {
		return detectedColor.green;
	}

	@Override
	public void periodic() {
		detectedColor = m_colorSensor.getColor();
		this.blue.setDouble(getBlue());
		this.red.setDouble(getRed());
		this.green.setDouble(getGreen());
	}
}
