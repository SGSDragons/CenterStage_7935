package org.firstinspires.ftc.teamcode.Utilities;

public class Constants {

    public static boolean unlockDriveSpeed = false;
    public static boolean fieldOrientation = true;

    //Hardware IDs
    public static final String backLeftDriveID = "backLeftDrive";
    public static final String backRightDriveID = "backRightDrive";
    public static final String frontLeftDriveID = "frontLeftDrive";
    public static final String frontRightDriveID = "frontRightDrive";

    public static final String armID = "arm";
    public static final String extenderID = "extender";
    public static final String intakeID = "intake";
    public static final String wristID = "wrist";

    public static final String armLimitSwitchID = "armLimit";
    public static final String extenderLimitSwitchID = "extenderLimit";


    //Motor Power Values
    public static double teleOPArmPower = 1.0;
    public static double autoIntakePower = 1.0;
    public static double teleOPWristIncrement = 0.1;


    public static double maxDrivePower = 0.5;

    //Drive Encoder/Position Values (FrontLeft Motor is Leader)
    public static final double driveMotorCPR = 28;
    public static final double driveGearRatio = 43/3;
    public static final double wheelDiameterInches = 75 / 25.4;
    public static final double driveMotorCPI = (driveMotorCPR * driveGearRatio) /
            (wheelDiameterInches * Math.PI);
    public static final double xDriveScaleFactor = 1.0;

    //Erector Encoder/Position Values
    public static final double maxErectorPositionInches = 36;
    public static final double erectorMotorCPR = 28;
    public static final double erectorGearRatio = 125;
    public static double erectorMotorRPI = 1.0;
    public static double erectorMotorCPI = (erectorMotorCPR * erectorGearRatio) * erectorMotorRPI;

    //Extender
    public static final double extenderMaxPositionInches = 36;
    public static final double extenderMotorCPR = 28;
    public static final double extenderGearRatio = 125;
    public static final double spoolRPI = 1.0;
    public static final double extenderMotorCPI = extenderMotorCPR * extenderGearRatio * spoolRPI;
    public static final double extenderZeroingPower = -0.5;


    //Arm Encoder/Position Values
    public static double armMotorCPR = 28;
    public static double armGearRatio = 60;
    public static int armMotor120Position = (int) ((120 / 360) * (armMotorCPR * armGearRatio));
    public static int armMotor90Position = (int) ((90 / 360) * (armMotorCPR * armGearRatio));

    public static final double armZeroingPower = -0.5;
    public static final int armZeroingAddition = 56;

    //Servo Position Values
    public static double wristMinPosition = 0.0;
    public static double wristMaxPosition = 1.0;

    //PID Values
    public static double driveK = 0.01;
    public static double driveI = 0.0;
    public static double driveD = 0.0;

    public static double turnK = 0.1;
    public static double turnI = 0.0;
    public static double turnD = 0.0;

    public static double armP = 0.1;
    public static double armI = 0.0;
    public static double armD = 0.0;
}
