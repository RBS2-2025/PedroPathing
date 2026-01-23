package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.movement.IMUDriving;

@TeleOp(name = "MoveTest")
public class MoveTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            IMUDriving imuDriving = new IMUDriving(hardwareMap,telemetry,gamepad1);
            while (opModeIsActive()) {
                // OpMode loop
                imuDriving.controlWithPad(IMUDriving.GamepadPurpose.WHOLE);
            }
        }
    }
}
