package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "WheelGalGal")
public class WheelGalGal extends LinearOpMode {
    DcMotor fl,fr;
    boolean active = false;
    boolean isAPressed = false;
    boolean isActive = false;

    @Override
    public void runOpMode() {

        fl = hardwareMap.dcMotor.get("lf");
        fr = hardwareMap.dcMotor.get("rf");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                // OpMode loop
                if(gamepad1.a && !isAPressed) {
                    active = !active;
                }
                if(active && !isActive){
                    isActive = true;
                    fl.setPower(1);
                    fr.setPower(1);

                }
                if(!active && isActive){
                    isActive = false;
                    fl.setPower(0);
                    fr.setPower(0);
                }


            }
        }
    }

}