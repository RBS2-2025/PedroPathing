package org.firstinspires.ftc.teamcode.Movement;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class ActionManaging_main {

    public DcMotor outtake, outtake2, intake;
    private ElapsedTime timer = new ElapsedTime();

    private boolean outtakeActive = false;

    public static double intakeDelay = 3;
    public static double intakePower = 1.0;
    public static double feedOnTime = 2.0;
    public static double feedOffTime = 3.0; // 2초 멈추기

    public ActionManaging_main(
            DcMotor outtake,
            DcMotor outtake2,
            DcMotor intake
    ) {
        this.outtake = outtake;
        this.outtake2 = outtake2;
        this.intake = intake;
    }

    // intake

    public void intake(double power) {
        intake.setPower(power);
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

        outtakeActive = false;
    }
}
