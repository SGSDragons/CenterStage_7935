package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.WristSubsystem;

public class AutoIntake {
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
    public AutoIntake(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem, WristSubsystem wristSubsystem, ElapsedTime runtime, Telemetry telemetry) {
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
        extenderSubsystem.extendToPosition(ExtenderSubsystem.Position.RETRACTED);

        initialRuntime = runtime.seconds();
        while (runtime.seconds() - initialRuntime < 0.5) {}

        armSubsystem.autoPositionArm(ArmSubsystem.Position.DOWN);
        wristSubsystem.autoAngleWrist(WristSubsystem.Positions.DOWN);

        initialRuntime = runtime.seconds();
        while (runtime.seconds() - initialRuntime < 0.5) {}

        intakeSubsystem.intake();
        drivetrainSubsystem.driveAutoOld(2, DrivetrainSubsystem.Directions.FORWARD);

        initialRuntime = runtime.seconds();
        while (runtime.seconds() - initialRuntime < 0.5) {}

        intakeSubsystem.stopMotor();
        telemetry.addData("Status", "Performed AutoIntake");
        telemetry.update();
    }
}
