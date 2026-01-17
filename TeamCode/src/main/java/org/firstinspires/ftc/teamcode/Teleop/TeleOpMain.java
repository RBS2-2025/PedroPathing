package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Movement.ActionManaging;
import org.firstinspires.ftc.teamcode.Movement.IMU_Driving;
import org.firstinspires.ftc.teamcode.Vision.vision;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleOpMain extends LinearOpMode {
    Servo lifting;
    DcMotor Turret_R, IntakeDc, fl, fr, rl, rr;
    DcMotorEx Turret_S;
    Limelight3A limelight;

    IMU imu;
    IMU_Driving imu_driving;
    ActionManaging action;
    vision visionModule = new vision();

    public static double intakePower = 1.0;
    public static double outtakePower = 1;
    public static final double FAR_OTPOWRE = 1;
    public static final double NEAR_OTPOWRE = 0.45;

    //public static double Turret_R_Speed = 0.15;
    private boolean In_wasPressed = false;
    private boolean InR_wasPressed = false;
    private boolean Out_wasPressed = false;
    private boolean Align_wasPressed = false;
    private boolean a_wasPressed = false;
    private boolean b_wasPressed = false;
    private boolean left_wasPressed = false;
    private boolean right_wasPressed = false;

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() {
        initialize();

        timer.reset();

        action = new ActionManaging(Turret_S, Turret_R, IntakeDc);
        imu_driving = new IMU_Driving(fl,fr,rl, rr, imu, telemetry,gamepad1);
        visionModule.VisionModule(hardwareMap, telemetry);

        imu_driving.init();
        imu_driving.getYaw();

        waitForStart();


        if (opModeIsActive()) {

            while (opModeIsActive()) {
                imu_driving.controlWithPad(IMU_Driving.GamepadPurpose.WHOLE);
                action.preheat();
                intakeR();
                intake();
                outtake();
                align();
                adjustOuttakePower();

                double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

                action.updateFlywheelPIDF(voltage);

                telemetry.addData("Outtake Power", outtakePower);

                telemetry.update();
            }
        }
    }

    /**
     * 인테이크
     */
    void intake(){
        // intake (gamepad2.a)
        if (gamepad2.a) {
            action.intake(intakePower);
            if (!In_wasPressed) In_wasPressed = true;
        }
        if (!gamepad2.a && In_wasPressed) {
            action.intake_stop();
            In_wasPressed = false;
        }
    }

    /**
     * 배출
     */
    void intakeR(){
        // intake reverse (gamepad2.x)
        if (gamepad2.x) {
            action.intake_r();
            if (!InR_wasPressed) InR_wasPressed = true;

        }
        if (!gamepad2.x && InR_wasPressed) {
            action.intake_stop();
            InR_wasPressed = false;
        }
    }


    void outtake(){
        // outtake (gamepad2.b)
        if (gamepad2.b) {
            if (!Out_wasPressed){
                timer.reset();
                Out_wasPressed = true;
                while(timer.time(TimeUnit.SECONDS) <= 1.5){
                    intakeR();
                    intake();
                    align();
                    adjustOuttakePower();
                    action.outtake();
                    if(!gamepad2.b){
                        break;
                    }
                }
                action.intake(intakePower);
                intakeR();
                intake();
                align();
                adjustOuttakePower();
            }
            else{
                action.outtake();
                action.intake(intakePower);
            }
        }

        if (!gamepad2.b && Out_wasPressed) {
            action.outtake_stop();
            Out_wasPressed = false;
            delay(1);
            action.preheat();
        }
    }
    void delay(double t){
        ElapsedTime rePreheatTimer = new ElapsedTime();
        rePreheatTimer.reset();
        while (rePreheatTimer.time(TimeUnit.SECONDS) <= t){
            telemetry.addData("status: ","delaying..");
            telemetry.update();
        }
        if(rePreheatTimer.time(TimeUnit.SECONDS) >= t){
            return;
        }
    }

    /**
     * 터렛 얼라인
     */
    void align(){
        //align
        if (gamepad2.right_bumper){
            visionModule.align(Turret_R,true,0.03,0.1,2.0);
            if (!Align_wasPressed) Align_wasPressed = true;
        }
        if (!gamepad2.right_bumper && Align_wasPressed){
            Turret_R.setPower(0);
            Align_wasPressed = false;
        }

        if (gamepad2.dpad_left) {
            Turret_R.setPower(-0.1);
            left_wasPressed = true;
        } else if (left_wasPressed) {
            Turret_R.setPower(0);
            left_wasPressed = false;
        }

        if (gamepad2.dpad_right) {
            Turret_R.setPower(0.1);
            right_wasPressed = true;
        } else if (right_wasPressed) {
            Turret_R.setPower(0);
            right_wasPressed = false;
        }
    }

    /**
     * 아웃테이크 파워 조정 (a,b,x,y)
     */
    void adjustOuttakePower(){
        // outtake Power 조정 g1.b - / g1.a +
        if (gamepad1.a && !a_wasPressed) {
            outtakePower += 0.05;
        }
        a_wasPressed = gamepad1.a;

        if (gamepad1.b && !b_wasPressed) {
            outtakePower -= 0.05;
        }
        b_wasPressed = gamepad1.b;

        // outtake Power 변경 g1.x 0.5 / g1.y 0.1
        if(gamepad1.x){
            outtakePower = NEAR_OTPOWRE;
        }
        if(gamepad1.y){
            outtakePower = FAR_OTPOWRE;
        }
    }


    void initialize() {

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        rl = hardwareMap.dcMotor.get("rl");
        rr = hardwareMap.dcMotor.get("rr");
        imu = hardwareMap.get(IMU.class,"imu");


        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        IntakeDc = hardwareMap.dcMotor.get("IntakeDc");
        Turret_S = hardwareMap.get(DcMotorEx.class, "Turret_S");
        Turret_R = hardwareMap.dcMotor.get("Turret_R");

        IntakeDc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IntakeDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret_S.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }




}