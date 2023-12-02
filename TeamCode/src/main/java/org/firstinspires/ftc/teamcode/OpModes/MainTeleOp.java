/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

@TeleOp(name="Main TeleOp", group="Linear OpMode")
//@Disabled
public class MainTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Initialize Subsystems
        DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(
                hardwareMap.get(DcMotor.class, Constants.backRightDriveID),
                hardwareMap.get(DcMotor.class, Constants.backLeftDriveID),
                hardwareMap.get(DcMotor.class, Constants.frontRightDriveID),
                hardwareMap.get(DcMotor.class, Constants.frontLeftDriveID),
                hardwareMap.get(IMU.class, "imu"),
                runtime, telemetry);
/*
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(
                hardwareMap.get(DcMotor.class, Constants.intakeID), runtime, telemetry);

 */
/*
        WristSubsystem wristSubsystem = new WristSubsystem(
                hardwareMap.get(Servo.class, Constants.wristID), runtime, telemetry);

 */
/*
        ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem(
                hardwareMap.get(DcMotor.class, Constants.extenderID),
                hardwareMap.get(TouchSensor.class, Constants.extenderLimitSwitchID),
                runtime, telemetry);

 */

        ArmSubsystem armSubsystem = new ArmSubsystem(
                hardwareMap.get(DcMotor.class, Constants.armID),
                hardwareMap.get(TouchSensor.class, Constants.armLimitSwitchID),
                runtime, telemetry);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drivetrainSubsystem.driveManual(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.a) {
                drivetrainSubsystem.resetGyro();
            }

            //intakeSubsystem.teleOPIntake(gamepad2.right_trigger, gamepad2.left_trigger);
            //wristSubsystem.manualAngleWrist(gamepad2.right_bumper, gamepad2.left_bumper);
            armSubsystem.ManualPositionArm(gamepad2.dpad_left, gamepad2.dpad_right);

            /*
            if (gamepad2.dpad_up) {
                wristSubsystem.autoAngleWrist(WristSubsystem.Positions.UP);
            }
            else if (gamepad2.dpad_down) {
                wristSubsystem.autoAngleWrist(WristSubsystem.Positions.DOWN);
            }

             */



            /*
            if (gamepad2.x) {
                extenderSubsystem.extendToPosition(ExtenderSubsystem.Position.RETRACTED);
            }
            else if (gamepad2.y) {
                extenderSubsystem.extendToPosition(ExtenderSubsystem.Position.HALFWAY);
            }
            else if (gamepad2.b) {
                extenderSubsystem.extendToPosition(ExtenderSubsystem.Position.EXTENDED);
            }

             */

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
