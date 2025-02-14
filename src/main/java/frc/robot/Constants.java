// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //LS = Limit Switchs

  public static class AlgaeIntakeConstants{
    //Motor ID's
    public static final int INTAKEID = 17;
    
    //Sensor ID's
    public static final int OPTICALID = 0;

    //Current Limiting
    public static final boolean CURRENTLIMIT = true;

    //Intake Max Speeds
    public static final double INTAKEMAXSPEED = 1.0;
    public static final double OUTTAKEMAXSPEED = 1.0;

    public static final int PIVOTID = 18;

    //Sensor ID's
    public static final int LSID = 3;

    //PID Constants and Other Important Variables
    public static final double KP = 0.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    public static final double TOLERANCE = 100.0;

    //Pivot Max Speed
    public static final double PIVOTMAXSPEED = 0.3;
  }

  public static class ElevatorConstants{
    //Motor ID's
    public static final int LIFTID = 14;

    //Sensor ID's
    // public static final int UPPERLSID = 2;
    public static final int BOTTOMLSID = 2;

    //PID Constants and Other Important Variables
    public static final double KP = 0.01;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    public static final double TOLERANCE = 5; 

    public static final double MAXSPEED = 0.1;
  }
}

