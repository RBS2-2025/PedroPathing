package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Movement.ActionManaging;
import org.firstinspires.ftc.teamcode.Movement.IMU_Driving;

@Configurable
@TeleOp
public class teleop extends LinearOpMode {
    DcMotor IntakeDc,fl, fr, rl, rr;
    IMU imu;
    IMU_Driving imu_driving;
    ActionManaging action;

    private boolean In_wasPressed = false;
    private boolean InR_wasPressed = false;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() {
        initialize();

        timer.reset();

        imu_driving = new IMU_Driving(fl,fr,rl, rr, imu, telemetry,gamepad1);
//        action = new ActionManaging(IntakeDc);

        imu_driving.init();
        imu_driving.getYaw();

        waitForStart();


        if (opModeIsActive()) {

            while (opModeIsActive()) {
                imu_driving.controlWithPad(IMU_Driving.GamepadPurpose.WHOLE);


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
            action.intake(1);
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


//        IntakeDc = hardwareMap.dcMotor.get("IntakeDc");
//
//        IntakeDc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        IntakeDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }




}



