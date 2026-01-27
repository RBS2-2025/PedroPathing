package org.firstinspires.ftc.teamcode.Movement;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class ActionManaging_main {

    public DcMotor outtake, outtake2, intake;
    public Servo blocker;
    private ElapsedTime timer = new ElapsedTime();

    private boolean outtakeActive = false;

    public static double intakeDelay = 3;
    public static double intakePower = 1.0;
    public static double feedOnTime = 2.0;
    public static double feedOffTime = 3.0; // 2초 멈추기
    public static double blockpos = 0;
    public static double unblockpos = 0;

    public ActionManaging_main(
            DcMotor outtake,
            DcMotor outtake2,
            DcMotor intake,
            Servo blocker
    ) {
        this.outtake = outtake;
        this.outtake2 = outtake2;
        this.intake = intake;
        this.blocker = blocker;
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
        block();
    }

    public void intake_stop() {
        intake.setPower(0);
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

       if (currentTime >= intakeDelay) {
           double timeSinceFeedStarted = currentTime - intakeDelay;
           double cycleTime = feedOnTime + feedOffTime;
           double timeInCycle = timeSinceFeedStarted % cycleTime;

           if (timeInCycle < feedOnTime) {
               intake.setPower(intakePower);
               unblock();
           } else {
               intake.setPower(0);
               //block();
           }

       } else {
           intake.setPower(0);
           //block();


       }
   }

    public void outtake_stop() {
        outtake.setPower(0);
        outtake2.setPower(0);

        intake.setPower(0);

        outtakeActive = false;
    }
}
