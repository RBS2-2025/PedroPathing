package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {
    DcMotor intake;
    boolean intakePressed = false;
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            intake = hardwareMap.dcMotor.get("IntakeDc");

            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop
                if(gamepad1.a && !intakePressed){
                    intake.setPower(0.7);
                    intakePressed = true;
                }
                if(gamepad1.b && !intakePressed){
                    intake.setPower(1);
                    intakePressed = true;
                }
                if (intakePressed && !gamepad1.a && !gamepad1.b){
                    intake.setPower(0);
                    intakePressed = false;
                }

            }
        }
    }
}
