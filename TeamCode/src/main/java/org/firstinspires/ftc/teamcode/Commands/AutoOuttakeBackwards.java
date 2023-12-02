package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.WristSubsystem;

public class AutoOuttakeBackwards {
    ElapsedTime runtime;
    Telemetry telemetry;
    double initialRuntime;

    //Declare Variables and Utilized Subsystems
    IntakeSubsystem intakeSubsystem;
    DrivetrainSubsystem drivetrainSubsystem;
    ArmSubsystem armSubsystem;
    ExtenderSubsystem extenderSubsystem;
    WristSubsystem wristSubsystem;

    //Constructor intakes subsystem objects and runtime.
    public AutoOuttakeBackwards(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem, WristSubsystem wristSubsystem, ElapsedTime runtime, Telemetry telemetry) {
        this.intakeSubsystem = intakeSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armSubsystem = armSubsystem;
        this.extenderSubsystem = extenderSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.runtime = runtime;
        this.telemetry = telemetry;

        runCommand();
    }

    //Command Below
    private void runCommand() {
        //Actions that require no execution time or pause should be listed as such:

        armSubsystem.autoPositionArm(ArmSubsystem.Position.UP);
        wristSubsystem.autoAngleWrist(WristSubsystem.Positions.UP);

        initialRuntime = runtime.seconds();
        while (runtime.seconds() - initialRuntime < 0.5) {}

        extenderSubsystem.extendToPosition(ExtenderSubsystem.Position.EXTENDED);

        initialRuntime = runtime.seconds();
        while (runtime.seconds() - initialRuntime < 0.5) {}

        intakeSubsystem.outtake();

        initialRuntime = runtime.seconds();
        while (runtime.seconds() - initialRuntime < 0.5) {}

        intakeSubsystem.stopMotor();
        telemetry.addData("Status", "Performed AutoOuttake");
        telemetry.update();
    }
}
