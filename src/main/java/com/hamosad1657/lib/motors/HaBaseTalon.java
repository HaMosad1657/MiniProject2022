// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.hamosad1657.lib.motors;

import com.hamosad1657.lib.HaUnits;

/** Add your docs here. */
abstract public class HaBaseTalon extends HaMotorController {
    public abstract void setSim(double value, HaUnits.Velocity type);
    public abstract void setSim(double value, HaUnits.Position type);
    public abstract double getSim(HaUnits.Velocity type);
    public abstract double getSim(HaUnits.Position type);
}
