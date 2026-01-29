package org.firstinspires.ftc.teamcode.Movement;

import static org.firstinspires.ftc.teamcode.TEST.pidf.F;
import static org.firstinspires.ftc.teamcode.TEST.pidf.F2;
import static org.firstinspires.ftc.teamcode.TEST.pidf.P;
import static org.firstinspires.ftc.teamcode.TEST.pidf.P2;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class ActionManaging_main {

    public DcMotor intake;
    public DcMotorEx outtake, outtake2;
    public Servo blocker,push;
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
            Servo push
    ) {
        this.outtake = outtake;
        this.outtake2 = outtake2;
        this.intake = intake;
        this.blocker = blocker;
        this.push = push;
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
   public void outtake(double power1, double power2) {
       if (!outtakeActive) {
           outtake.setPower(power1);
           outtake2.setPower(power2);

           timer.reset();
           outtakeActive = true;
       }

       double currentTime = timer.seconds();
       if (currentTime >= blockerdelay) {
           unblock();
       }

       if (currentTime >= intakeDelay) {
           double timeSinceFeedStarted = currentTime - intakeDelay;
           double cycleTime = feedOnTime + feedOffTime;
           double timeInCycle = timeSinceFeedStarted % cycleTime;

           if (timeInCycle < feedOnTime) {
               intake.setPower(intakePower);
           } else {
               intake.setPower(0);
           }
       } else {
           intake.setPower(0);
       }
   }

    public void outtake_stop() {
        outtake.setPower(0);
        outtake2.setPower(0);

        intake.setPower(0);
        push.setPosition(0.5);
        block();
        outtakeActive = false;
    }
}
