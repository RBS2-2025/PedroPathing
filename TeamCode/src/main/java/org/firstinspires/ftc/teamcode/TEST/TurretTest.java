package org.firstinspires.ftc.teamcode.TEST;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.movement.IMUDriving;
import org.firstinspires.ftc.teamcode.utils.PanelsHelper;

import java.util.concurrent.TimeUnit;
@Configurable
@TeleOp(name = "TurretTest",group = "test")
public class TurretTest extends LinearOpMode {
    public static double kP = 10;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 10;
    PIDFCoefficients coefficients;
    public static int PREHEAT_VELOCITY = 1000;
    public static int VELOCITY = 2000;
    public static boolean USE_PIXEL = false;
    int targetVeocity;
    int unit = 100;
    DcMotorEx outtakeMotor;
    DcMotor intake;
    PanelsHelper panels;
    boolean aPressed = false;
    boolean outtakeResting = false;
    boolean intakePressed;
    Timer outtakeTimer;


    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run

//            TurretControl turret = new TurretControl(hardwareMap,telemetry,true,imu);

            outtakeMotor = hardwareMap.get(DcMotorEx.class,"Turret_S");
            panels = new PanelsHelper(this);
            targetVeocity = PREHEAT_VELOCITY;
            outtakeTimer = new Timer();
            outtakeTimer.resetTimer();

            intake = hardwareMap.dcMotor.get("IntakeDc");

            IMUDriving imuDriving = new IMUDriving(hardwareMap,telemetry,gamepad1);
            while (opModeIsActive()) {
                // OpMode loop
                imuDriving.controlWithPad(IMUDriving.GamepadPurpose.WHOLE);
                //
                coefficients = new PIDFCoefficients(kP,kI,kD,kF*(12/hardwareMap.voltageSensor.iterator().next().getVoltage()));
                outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coefficients);
//                turret.align(0.1,USE_PIXEL);
//                double distance = turret.getDistance();

                if(gamepad2.dpad_up){
                    VELOCITY += unit;
                }
                if(gamepad2.dpad_down){
                    VELOCITY -= unit;
                    VELOCITY = Math.max(VELOCITY,0);
                }
                if(gamepad2.dpad_left){
                    unit /= 10;
                    unit = Math.max(unit, 1);
                }
                if(gamepad2.dpad_right){
                    unit *= 10;
                }
                if(gamepad2.b && !aPressed){
                    aPressed = true;
                    targetVeocity = VELOCITY;
                    outtakeMotor.setVelocity(targetVeocity);
                }
                if(!gamepad2.b && aPressed){
                    aPressed = false;
                    outtakeResting = true;
                    outtakeMotor.setPower(0);
                    outtakeTimer.resetTimer();
                }
                if(!gamepad2.b && outtakeResting){
                    if(outtakeTimer.getElapsedTimeSeconds() > 1){
                        outtakeResting = false;
                        targetVeocity = PREHEAT_VELOCITY;
                        outtakeMotor.setVelocity(targetVeocity);
                    }
                }
                if(gamepad2.a && !intakePressed){
                    intake.setPower(-0.6);
                    intakePressed = true;
                }
                if (intakePressed && !gamepad2.a){
                    intake.setPower(0);
                    intakePressed = false;
                }

//                panels.addData("distance: ",distance==-1?"error":distance);
                panels.addData("target velocity: ",targetVeocity);
                panels.update();
            }
        }
    }
}
