// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.utils;

/** Add your docs here. */
public class Debug {

    public static final String DT_DEBUG_TABLE = "debug.DriveTrain";
    public static final String DT_LEFT_VOLTS = "l_volt";
    public static final String DT_RIGHT_VOLTS = "r_volt";
    public static final String DT_X = "x";
    public static final String DT_Y = "y";
    public static final String DT_ROT = "rot";
    public static final String DT_L_DIST = "l_dist";
    public static final String DT_R_DIST = "r_dist";
    public static final String DT_WHEEL_SPEEDS = "wheel_speeds";

    public static final String DT_L_REF = "l_ref";
    public static final String DT_R_REF = "r_ref";
    public static final String DT_L_MEASURE = "l_measure";
    public static final String DT_R_MEASURE = "r_measure";

    // public static final String DT_ = "";
    // public static final String DT_ = "";
    // public static final String DT_ = "";
    // public static final String DT_ = "";
    
    //private static final NetworkTable nt;

    public Debug() {
        //= NetworkTableInstance.getDefault().getTable("debug.DriveTrain");
    }
    //public getDrive

    public static double round(double v) {
        v = v * 100;
        v = Math.floor(v);
        v = v / 100;
        return v;
      }
}
