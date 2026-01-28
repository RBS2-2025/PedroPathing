package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Movement.Mecanum_Driving;

@TeleOp
public class drive extends LinearOpMode {
    Mecanum_Driving drive;

    @Override
    public void runOpMode() {

        drive = new Mecanum_Driving(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        }

        drive.stop();
    }
}
