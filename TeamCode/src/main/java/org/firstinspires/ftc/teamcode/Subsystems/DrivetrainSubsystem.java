package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.MiniPID;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class DrivetrainSubsystem {
    private ElapsedTime runtime;
    private Telemetry telemetry;

    //Drive Powers
    private double backLeftPower;
    private double backRightPower;
    private double frontLeftPower;
    private double frontRightPower;

    //Drive Motors
    private DcMotor backRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor frontLeftDrive;

    private IMU imu;

    private MiniPID drivePID = new MiniPID(Constants.driveK, Constants.driveI, Constants.driveD);

    private MiniPID xDrivePID = new MiniPID(Constants.driveK, Constants.driveI, Constants.driveD);
    private MiniPID yDrivePID = new MiniPID(Constants.driveK, Constants.driveI, Constants.driveD);

    private MiniPID turnPID = new MiniPID(Constants.turnK, Constants.turnI, Constants.turnD);

    public enum Directions {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    public DrivetrainSubsystem(DcMotor backRightDrive, DcMotor backLeftDrive, DcMotor frontRightDrive,
                               DcMotor frontLeftDrive, IMU imu, ElapsedTime runtime, Telemetry telemetry) {
        this.backRightDrive = backRightDrive;
        this.backLeftDrive = backLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.frontLeftDrive = frontLeftDrive;
        this.imu = imu;
        this.runtime = runtime;
        this.telemetry = telemetry;

        initialize();
    }

    //Initializes Hardware
    private void initialize() {
        //Drive Motors
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        if (Constants.unlockDriveSpeed) {
            drivePID.setOutputLimits(-1.0, 1.0);
            turnPID.setOutputLimits(-1.0, 1.0);
        }
        else {
            drivePID.setOutputLimits(-0.25, 0.25);
            turnPID.setOutputLimits(-0.25, 0.25);
        }
        boolean drivePIDReversed = false;
        drivePID.setDirection(drivePIDReversed);

        xDrivePID.setOutputLimits(-0.5, 0.5);
        yDrivePID.setOutputLimits(-0.5, 0.5);
    }

    //General Drive / Motor Methods

    //Drive Not Utilizing Field Orientation
    protected void drive(double shuffleDrive, double forwardDrive, double turnDrive) {

        if (Constants.unlockDriveSpeed) {
            frontLeftPower = Range.clip(shuffleDrive + forwardDrive + turnDrive, -1.0, 1.0);
            frontRightPower = Range.clip(-shuffleDrive + forwardDrive - turnDrive, -1.0, 1.0);
            backLeftPower = Range.clip(-shuffleDrive + forwardDrive + turnDrive, -1.0, 1.0);
            backRightPower = Range.clip(shuffleDrive + forwardDrive - turnDrive, -1.0, 1.0);
        }
        else {
            frontLeftPower = Range.clip(shuffleDrive + forwardDrive + turnDrive, -Constants.maxDrivePower, Constants.maxDrivePower);
            frontRightPower = Range.clip(-shuffleDrive + forwardDrive - turnDrive, -Constants.maxDrivePower, Constants.maxDrivePower);
            backLeftPower = Range.clip(-shuffleDrive + forwardDrive + turnDrive, -Constants.maxDrivePower, Constants.maxDrivePower);
            backRightPower = Range.clip(shuffleDrive + forwardDrive - turnDrive, -Constants.maxDrivePower, Constants.maxDrivePower);
        }

        moveMotors();
    }

    //Drive Utilizing Field Orientation
    protected void drive(double shuffleDrive, double forwardDrive, double turnDrive, double a, double b) {

        frontLeftPower = turnDrive + (shuffleDrive * b) + (forwardDrive * a);
        frontRightPower = -turnDrive + (-shuffleDrive * a) + (forwardDrive * b);
        backLeftPower = turnDrive + (-shuffleDrive * a) + (forwardDrive * b);
        backRightPower = -turnDrive + (shuffleDrive * b) + (forwardDrive * a);

        if (Constants.unlockDriveSpeed) {
            frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
            frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
            backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
            backRightPower = Range.clip(backRightPower, -1.0, 1.0);
        }
        else {
            frontLeftPower = Range.clip(frontLeftPower, -Constants.maxDrivePower, Constants.maxDrivePower);
            frontRightPower = Range.clip(frontRightPower, -Constants.maxDrivePower, Constants.maxDrivePower);
            backLeftPower = Range.clip(backLeftPower, -Constants.maxDrivePower, Constants.maxDrivePower);
            backRightPower = Range.clip(backRightPower, -Constants.maxDrivePower, Constants.maxDrivePower);
        }

        moveMotors();
    }

    protected void moveMotors() {
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
    }

    //Stop Motors
    protected void stopMotors() {
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
    }

    //Specific Actions

    //Calculates Joystick Inputs into Drive Commands
    public void joystickDrive(double leftStickX, double leftStickY, double rightStickX) {
        if (Constants.fieldOrientation) {
            double angleRadians = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double a = cos(angleRadians) - sin(angleRadians);
            double b = cos(angleRadians) + sin(angleRadians);
            drive(leftStickX, -leftStickY, rightStickX, a, b);
        }
        else {
            drive(leftStickX, -leftStickY, rightStickX);
        }
    }

    //Intakes a 2D RealVector with an X Position and Y Position in Inches. Drives the robot along the vector.
    public void autoDrive(double x, double y) {
        //Reset PIDs from last use.
        xDrivePID.reset();
        yDrivePID.reset();

        //Set up Vector for Future Transformations
        double [] desiredMovementVectorData = {x, y};
        RealVector desiredMovementVector = new ArrayRealVector(desiredMovementVectorData);


        //Calculate Desired X and Y Movement
        int xMovement = (int) (desiredMovementVector.getEntry(0) * Constants.xDriveScaleFactor * Constants.driveMotorCPI);
        int yMovement = (int) (desiredMovementVector.getEntry(1) * Constants.driveMotorCPI);

        int zeroPositionX = frontLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition();
        int currentPositionX = zeroPositionX;

        int zeroPositionY = frontRightDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition();
        int currentPositionY = zeroPositionY;

        int xTarget = zeroPositionX + xMovement;
        int yTarget = zeroPositionY + yMovement;

        int xDistance = xTarget - currentPositionX;
        float xInitialSign = signum(xDistance);

        int yDistance = yTarget - currentPositionY;
        float yInitialSign = signum(yDistance);

        //Initialize PIDs and Used Variables
        xDrivePID.setSetpoint(0.0);
        yDrivePID.setSetpoint(0.0);

        double xDrive;
        double yDrive;

        boolean xPositionReached = false;
        boolean yPositionReached = false;
        boolean positionReached = false;

        while(!positionReached) {
            //Calculate Distance to X Target Position
            if (xPositionReached) {
                xDistance = 0;
            }
            else {
                xDistance = xTarget - currentPositionX;
            }

            //Calculate Distance to Y Target Position
            if (yPositionReached) {
                yDistance = 0;
            }
            else {
                yDistance = yTarget - currentPositionY;
            }

            //Check if the positions were reached via the distances' change in sign.
            if (!xPositionReached && signum(xDistance) != xInitialSign) {
                xPositionReached = true;
            }

            if (!yPositionReached && signum(yDistance) != yInitialSign) {
                yPositionReached = true;
            }

            //If Both Positions Reached, End the Loop
            if (xPositionReached && yPositionReached) {
                positionReached = true;
            }

            //Drive the robot based off these distance values.
            xDrive = xDrivePID.getOutput(xDistance);
            yDrive = yDrivePID.getOutput(yDistance);
            drive(xDrive, yDrive, 0.0);

            //Calculate new CurrentPositions
            currentPositionX = frontLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition();
            currentPositionY = frontRightDrive.getCurrentPosition() + backLeftDrive.getCurrentPosition();
        }
        stopMotors(); //Stops motors once position is reached.
    }

    //Turns the robot a certain degree amount either left or right.
    public void autoTurn(double turnAngleDegrees, Directions directions) {
        turnPID.reset(); //Reset PID from Last Use

        double turnAngleRadians = turnAngleDegrees * (Math.PI/180); // Convert turn angle to radians.
        double startAngle = getAdjustedIMUAngle(); // Initial yaw of the robot.

        double [] initialVectorData = {0, 1};
        RealVector initialVector = new ArrayRealVector(initialVectorData); // Initial angle unit vector. Represents robot facing "forwards" in the y direction on a cartesian plane.

        RealMatrix fullRotationMatrix; // Transformation matrix that applies the full desired rotation. Pre-multiplying with a vector results in a new vector with that transformation applied.
        try {
            fullRotationMatrix = calculateRotationMatrix(turnAngleRadians, directions);
        }
        catch (Exception e) {
            telemetry.addData("Error", "Failed to calculate rotation matrix, incompatible direction");
            return;
        }
        RealVector targetVector = fullRotationMatrix.preMultiply(initialVector); // Target angle unit vector. Represents that robot's target orientation on the x-y plane.

        double time = runtime.nanoseconds();
        double lastTime = time - 1; // 1 is initially subtracted to avoid dividing by zero when calculating derivative or second derivative.
        double deltaTime;

        double distance; // "Distance" the current robot's orientation is from the target orientation.
                         // Represents the difference between the target unit vector's scalar (of 1) and the current angle's scalar when projected onto the target vector.
                         // Ranges from 0 to 2, with 0 being when the two vectors perfectly align and 2 being when they point opposite of each other.
        try {
            distance = calculateProjectedAngleVectorDistance(targetVector, getAdjustedIMUAngle(), directions);
        }
        catch (Exception e) {
            telemetry.addData("Error", "Failed to calculate angle distance, incompatible direction");
            return;
        }

        double lastDistance = distance;
        double deltaDistance;

        double derivative = 0;
        double lastDerivative = derivative;
        double deltaDerivative;

        double secondDerivative;

        turnPID.setSetpoint(0.0);
        double turnPower;

        boolean approachingTarget = false;
        boolean targetReached = false;

        while (!targetReached) {
            try {
                distance = calculateProjectedAngleVectorDistance(targetVector, getAdjustedIMUAngle(), directions);
            }
            catch (Exception e) {
                telemetry.addData("Error", "Failed to calculate angle distance, incompatible direction");
                targetReached = true;
            }

            if (approachingTarget) {
                turnPower = turnPID.getOutput(distance);
                if (0.05 >= turnPower && turnPower >= -0.05) {
                    targetReached = true;
                }

                drive(0.0, 0.0, turnPower);
            }

            else {
                deltaTime = time - lastTime;
                deltaDistance = distance - lastDistance;

                derivative = deltaDistance / deltaTime;
                deltaDerivative = derivative - lastDerivative;

                secondDerivative = deltaDerivative / deltaTime;

                if (signum(secondDerivative) == -1 && signum(derivative) == -1) {
                    approachingTarget = true;
                }

                switch (directions) {
                    case LEFT:
                        turnPower = -0.5;
                        break;

                    case RIGHT:
                        turnPower = 0.5;
                        break;

                    default:
                        return;
                }
                drive(0.0, 0.0, turnPower);

                lastDistance = distance;
                lastTime = time;
                lastDerivative = derivative;
            }
        }

        stopMotors();
    }

    protected RealMatrix calculateRotationMatrix(double angleRadians, Directions directions) {
        double [][] rotationMatrixData;
        RealMatrix rotationMatrix;

        switch (directions) {
            case LEFT:
                rotationMatrixData = new double[][] { {cos(angleRadians), sin(angleRadians)}, {-sin(angleRadians), cos(angleRadians)} };
                rotationMatrix = new Array2DRowRealMatrix(rotationMatrixData);
                return rotationMatrix;

            case RIGHT:
                rotationMatrixData = new double[][] { {cos(angleRadians), -sin(angleRadians)}, {sin(angleRadians), cos(angleRadians)} };
                rotationMatrix = new Array2DRowRealMatrix(rotationMatrixData);
                return rotationMatrix;

            default:
                throw new RuntimeException("Invalid Turn Direction, Cannot Calculate Rotation");
        }
    }

    protected double calculateProjectedAngleVectorDistance(RealVector targetVector, double startAngle, Directions directions) {
        double currentAngle = (getAdjustedIMUAngle() - startAngle) % (2 * Math.PI);

        double [] initialVectorData = {0, 1};
        RealVector initialVector = new ArrayRealVector(initialVectorData);

        RealMatrix rotationMatrix = calculateRotationMatrix(currentAngle, directions);

        RealVector currentVector = rotationMatrix.preMultiply(initialVector);
        double distance = 1 - (targetVector.dotProduct(currentVector));
        return distance;
    }

    //Converts the -PI to PI IMU readings into 0 to 2PI angle values (following unit circle directions, counterclockwise increases angle)
    protected double getAdjustedIMUAngle() {
        double currentAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (signum(currentAngle) == -1) {
            return abs(currentAngle);
        }
        else {
            currentAngle = currentAngle - (2 * (currentAngle - Math.PI));
            return currentAngle;
        }
    }

    public void driveAutoOld(double distanceInches, Directions direction) {
        drivePID.reset();
        int startPosition = backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()
                + frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition();
        int currentPosition = startPosition;

        int targetPosition;
        double motorPower;
        boolean motorsActive = true;

        switch (direction) {
            case FORWARD:
                targetPosition = (int) (4 * Constants.driveMotorCPI * distanceInches) + startPosition;
                break;
            case BACKWARD:
                targetPosition = (int) (4 * Constants.driveMotorCPI * distanceInches) - startPosition;
                break;
            default:
                return;
        }

        drivePID.setSetpoint(targetPosition);
        double startTime = runtime.seconds();
        //Need to make sure that this will actually always stop eventually... don't want any infinite loops.
        while (motorsActive) {
            motorPower = drivePID.getOutput(currentPosition);

            double derivative = drivePID.derivativeOfError;
            if (-1 < derivative && derivative < 1 & runtime.seconds() - startTime > 1) {
                motorsActive = false;
            }

            if (runtime.seconds() - startTime > 8) {
                motorsActive = false;
            }

            drive(0.0, motorPower, 0.0);
            currentPosition = backLeftDrive.getCurrentPosition() + backRightDrive.getCurrentPosition()
                    + frontLeftDrive.getCurrentPosition() + frontRightDrive.getCurrentPosition();
        }
        //Once motorsActive is false, stop motors.
        stopMotors();
    }

    public void autoTurnOld(double turnAngleDegrees, Directions directions) {
        turnPID.reset();
        double turnAngleRadians = turnAngleDegrees * (Math.PI / 180);
        turnAngleRadians = Range.clip(turnAngleRadians, -Math.PI, Math.PI);
        double startAngle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double currentAngle = startAngle;
        double targetAngle;
        boolean motorsActive = true;
        boolean overshoot = false;
        double distance;
        double turnPower;
        boolean turningRight;
        double startTime = runtime.seconds();


        switch (directions) {
            case RIGHT:
                targetAngle = startAngle + turnAngleRadians;
                currentAngle += 0.01;
                turningRight = true;
                break;

            case LEFT:
                targetAngle = startAngle - turnAngleRadians;
                currentAngle -= 0.01;
                turningRight = false;
                break;

            default:
                return;
        }

        if (targetAngle < 0) {
            targetAngle += 2*Math.PI;
        }
        else if (targetAngle >= 2*Math.PI) {
            targetAngle -= 2*Math.PI;
        }

        turnPID.setSetpoint(0.0);
        while (motorsActive) {
            distance = calculateAngleDistance(currentAngle, targetAngle, turningRight, overshoot);
            turnPower = turnPID.getOutput(distance);

            if (Math.abs(turnPID.derivativeOfError) >= Math.PI && !overshoot) {
                distance = calculateAngleDistance(currentAngle, targetAngle, turningRight, true);
                turnPID.reset();
                turnPower = turnPID.getOutput(distance);

                overshoot = true;
            }

            else if (Math.abs(turnPID.derivativeOfError) <= 0.005 && overshoot) {
                motorsActive = false;
            }

            if (runtime.seconds() - startTime > 8) {
                motorsActive = false;
            }

            drive(0.0, 0.0, turnPower * (turningRight ? 1 : -1));
            currentAngle = getAngle();
        }
        stopMotors();
    }

    public double calculateAngleDistance(double currentAngle, double targetAngle, boolean turningRight, boolean overshoot) {
        double distance;
        if (overshoot) {
            distance = targetAngle - currentAngle;
        }
        else {
            distance = targetAngle + (turningRight ? -currentAngle : (-2 * targetAngle) + currentAngle);
        }
        return distance;
    }

    public void resetGyro() {
        imu.resetYaw();
    }


    //Get Methods
    public double getBackLeftPower() {
        return backLeftDrive.getPower();
    }

    public double getBackRightPower() {
        return backRightDrive.getPower();
    }

    public double getFrontLeftPower() {
        return frontLeftDrive.getPower();
    }

    public double getFrontRightPower() {
        return  frontRightDrive.getPower();
    }

    public double getAngle() {
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public int[] getDriveMotorCounts() {
        return new int[] {backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition(),
                frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition()};
    }
}
