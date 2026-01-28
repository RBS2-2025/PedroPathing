package org.firstinspires.ftc.teamcode.TEST;

import android.content.OperationApplicationException;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
@TeleOp
public class pidf extends OpMode {
    private static final double NOMINAL_VOLTAGE = 12.0;

    public DcMotorEx outtake, outtake2 ;
    public static double SHOOTING_VELOCITY = 2000;
    public static double PREHEAT_VELOCITY  = 1000;

    private boolean Outtake_wasPressed = false;

    double curTargetVelocity = PREHEAT_VELOCITY;
    public static double F = 0; //15.5
    public static double P = 0;
    public static double I = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001};

    int stepIndex = 1;

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        outtake2.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,I,0, F);
        outtake2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");

    }

    @Override
    public void loop() {
        if(gamepad1.yWasPressed()) {
            curTargetVelocity = SHOOTING_VELOCITY;
        }
        if(gamepad1.xWasPressed()) {
            curTargetVelocity = PREHEAT_VELOCITY;
        }
        if(gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,I,0, F*(12/hardwareMap.voltageSensor.iterator().next().getVoltage()));
        outtake2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set Velocity
        outtake2.setVelocity(curTargetVelocity);
        outtake.setVelocity(curTargetVelocity);

        double curVelocity2 = outtake2.getVelocity();
        double curVelocity = outtake2.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity",curTargetVelocity);
        telemetry.addData("Current Velocity","%.2f",curVelocity);
        telemetry.addData("Current Velocity2","%.2f",curVelocity2);
        telemetry.addData("Error","%.2f",error);
        telemetry.addLine("------------------------");
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)",P);
        telemetry.addData("Tuning F","%.4f (D-Pad L/R)",F);
        telemetry.addData("Step Size","%.4f (B button)",stepSizes[stepIndex]);

        telemetry.update();

    }

}