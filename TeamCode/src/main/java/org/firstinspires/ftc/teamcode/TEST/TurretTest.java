package org.firstinspires.ftc.teamcode.TEST;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.movement.IMUDriving;
import org.firstinspires.ftc.teamcode.utils.PanelsHelper;

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "TurretTest",group = "test")
public class TurretTest extends LinearOpMode {
    public static double kP = 800;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 25;
    PIDFCoefficients coefficients;
    public static int PREHEAT_VELOCITY = 1000;
    public static int VELOCITY = 2100;
    public static boolean USE_PIXEL = false;
    int targetVeocity;
    int unit = 100;
    DcMotorEx outtakeMotor;
    DcMotor intake;
    PanelsHelper panels;
    boolean bPressed = false;
    boolean outtakeResting = false;
    boolean intakePressed = false;
    ElapsedTime outtakeTimer;
    ElapsedTime feedTimer;


    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run


            outtakeMotor = hardwareMap.get(DcMotorEx.class,"Turret_S");
            panels = new PanelsHelper(this);
            targetVeocity = PREHEAT_VELOCITY;
            outtakeTimer = new ElapsedTime();
            outtakeTimer.reset();
            feedTimer = new ElapsedTime();
            feedTimer.reset();
            intake = hardwareMap.dcMotor.get("IntakeDc");

            while (opModeIsActive()) {
                // OpMode loop
                coefficients = new PIDFCoefficients(kP,kI,kD,kF*(12/hardwareMap.voltageSensor.iterator().next().getVoltage()));
                outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coefficients);

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
                if(gamepad2.b && !bPressed){
                    bPressed = true;
                    targetVeocity = VELOCITY;
                    feedTimer.reset();
                }
                if(bPressed){
                    if(feedTimer.time(TimeUnit.SECONDS) > 1.5){
                        intake.setPower(-1);
                    }
                }
                if(!gamepad2.b && bPressed && !outtakeResting){
                    bPressed = false;
                    outtakeResting = true;
                    outtakeMotor.setPower(0);
                    intake.setPower(0);
                    outtakeTimer.reset();
                }
                if(outtakeResting){
                    if(outtakeTimer.time(TimeUnit.SECONDS) > 3){
                        outtakeResting = false;
                        targetVeocity = PREHEAT_VELOCITY;
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
                if(!outtakeResting){
                    panels.addData("target velocity: ",targetVeocity);
                    outtakeMotor.setVelocity(targetVeocity);
                }
                else{
                    panels.addData("target velocity: ", "RESTING...");
                }

                panels.addData("current velocity:", outtakeMotor.getVelocity());
                panels.update();
            }
        }
    }
}