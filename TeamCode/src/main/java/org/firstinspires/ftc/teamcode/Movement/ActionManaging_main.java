package org.firstinspires.ftc.teamcode.Movement;

import static org.firstinspires.ftc.teamcode.TEST.pidf.F;
import static org.firstinspires.ftc.teamcode.TEST.pidf.F2;
import static org.firstinspires.ftc.teamcode.TEST.pidf.P;
import static org.firstinspires.ftc.teamcode.TEST.pidf.P2;
import static org.firstinspires.ftc.teamcode.Teleop.main2.curTargetVelocity;
import static org.firstinspires.ftc.teamcode.Teleop.main2.curTargetVelocity2;
import static org.firstinspires.ftc.teamcode.Teleop.main2.pushpos;
import static org.firstinspires.ftc.teamcode.Teleop.main2.unpushpos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class ActionManaging_main {

    public DcMotor intake;
    public DcMotorEx outtake, outtake2;
    public Servo blocker,push;
    public HardwareMap hardwareMap;
    private ElapsedTime timer = new ElapsedTime();

    private boolean outtakeActive = false;

    public static double intakeDelay = 3;
    public static double intakePower = 0.8;
    public static double feedOnTime = 1.3;
    public static double feedOffTime = 0.25; // 2초 멈추기
    public static double blockerdelay = 2.0;
    public static double blockpos = 0.35;
    public static double unblockpos = 0.62;
    public static double tried = 0;

    // 0.62

    public ActionManaging_main(
            DcMotorEx outtake,
            DcMotorEx outtake2,
            DcMotor intake,
            Servo blocker,
            Servo push,
            HardwareMap hardwareMap
    ) {
        this.outtake = outtake;
        this.outtake2 = outtake2;
        this.intake = intake;
        this.blocker = blocker;
        this.push = push;
        this.hardwareMap = hardwareMap;
    }
    public void block() {
        blocker.setPosition(blockpos);
    }
    public void unblock() {
        blocker.setPosition(unblockpos);
    }
    // intake

    public void intake(double power) {
        intake.setPower(power);
//        block();
    }

    public void intake_stop() {
        intake.setPower(0);
    }
    // pidf
    public void updateFlywheelPIDF(double batteryVoltage) {
        double compensatedF = F * (12 / batteryVoltage);

        PIDFCoefficients pidf = new PIDFCoefficients(
                P, 0, 0, compensatedF
        );
        outtake.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        double compensatedF2 = F2 * (12 / batteryVoltage);

        PIDFCoefficients pidf2 = new PIDFCoefficients(
                P2, 0, 0, compensatedF2
        );
        outtake2.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf2
        );
    }
   // outtake
    public void outtake(double v1,double v2){
        outtake.setVelocity(v1);
        outtake2.setVelocity(v2);
        updateFlywheelPIDF(hardwareMap.voltageSensor.iterator().next().getVoltage());
    }

    public void outtake_stop(){
        outtake.setVelocity(0);
        outtake2.setVelocity(0);
    }
    public void outtake1(){ // 구상중
        block();
        intake(1.0);
        //1초
        intake_stop();
        unblock();
        outtake(1000,1000);
        //4초
        outtake(2000,2000);
        push.setPosition(pushpos);
        push.setPosition(unpushpos);

        //4초

    }

}
