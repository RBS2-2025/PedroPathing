package org.firstinspires.ftc.teamcode.TEST;

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
public class IMU_Driving_test extends LinearOpMode {
    Servo lifting;
    DcMotor fl, fr, rl, rr;
    Limelight3A limelight;

    IMU imu;
    IMU_Driving imu_driving;
    ActionManaging action;
  //  vision visionModule = new vision();

//    public static double intakePower = 1.0;
//    public static double outtakePower = 1;
//    public static final double FAR_OTPOWRE = 1;
//    public static final double NEAR_OTPOWRE = 0.45;
//
//    //public static double Turret_R_Speed = 0.15;
//    private boolean In_wasPressed = false;
//    private boolean InR_wasPressed = false;
//    private boolean Out_wasPressed = false;
//    private boolean Align_wasPressed = false;
//    private boolean a_wasPressed = false;
//    private boolean b_wasPressed = false;
//    private boolean left_wasPressed = false;
//    private boolean right_wasPressed = false;

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() {
        initialize();

        timer.reset();

        imu_driving = new IMU_Driving(fl,fr,rl, rr, imu, telemetry,gamepad1);
       // visionModule.VisionModule(hardwareMap, telemetry);

        imu_driving.init();
        imu_driving.getYaw();

        waitForStart();


        if (opModeIsActive()) {

            while (opModeIsActive()) {
                imu_driving.controlWithPad(IMU_Driving.GamepadPurpose.WHOLE);
                //action.preheat();
                //intake();


                double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

                //action.updateFlywheelPIDF(voltage);

               // telemetry.addData("Outtake Power", outtakePower);

                telemetry.update();
            }
        }
    }

    /**
     * 인테이크
     */
    /**
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
**/



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


    }




}