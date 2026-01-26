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
    public static double kP = 450;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 15;
    PIDFCoefficients coefficients;
    public static int PREHEAT_VELOCITY = 1000;
    public static int VELOCITY = 1500;
    public static boolean USE_PIXEL = false;
    int targetVeocity;
    int unit = 100;
    DcMotorEx outtakeMotor;
    DcMotor intake;
    PanelsHelper panels;
    boolean bPressed = false;
    boolean outtakeResting = false;
    boolean intakePressed;
    ElapsedTime outtakeTimer;


    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            // Pre-run

//            TurretControl turret = new TurretControl(hardwareMap,telemetry,true,imu);

            outtakeMotor = hardwareMap.get(DcMotorEx.class,"Turret_S");
            panels = new PanelsHelper(this);
            targetVeocity = PREHEAT_VELOCITY;
            outtakeTimer = new ElapsedTime();
            outtakeTimer.reset();

            intake = hardwareMap.dcMotor.get("IntakeDc");

//            IMUDriving imuDriving = new IMUDriving(hardwareMap,telemetry,gamepad1);
            while (opModeIsActive()) {
                // OpMode loop
//                imuDriving.controlWithPad(IMUDriving.GamepadPurpose.WHOLE);
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
                if(gamepad2.b && !bPressed){
                    bPressed = true;
                    targetVeocity = VELOCITY;
                }
                if(!gamepad2.b && bPressed && !outtakeResting){
                    bPressed = false;
                    outtakeResting = true;
                    outtakeMotor.setPower(0);
                    outtakeTimer.reset();
                }
                if(outtakeResting){
                    if(outtakeTimer.time(TimeUnit.SECONDS) > 2 + kP/100){
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

//                panels.addData("distance: ",distance==-1?"error":distance);
                panels.addData("current velocity:", outtakeMotor.getVelocity());
                panels.update();
            }
        }
    }
}
