package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@Autonomous
public class leaving extends LinearOpMode {

    private DcMotor fl, rl, fr, rr;
    private ElapsedTime runtime = new ElapsedTime();

    private enum State {
        START_DELAY,
        MOVING_FORWARD,
        STOPPING,
        DONE
    }

    private State currentState = State.START_DELAY;

    public static double START_WAIT_SECONDS = 0;
    public static double MOVE_FORWARD_SECONDS = 2.3;
    public static double FORWARD_POWER = 0.3;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        rl  = hardwareMap.get(DcMotor.class, "rl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rr = hardwareMap.get(DcMotor.class, "rr");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready to run (Time Based)");
        telemetry.addData(">", String.format("Will move for %.1f seconds", MOVE_FORWARD_SECONDS));
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        runtime.reset();

        while (opModeIsActive() && currentState != State.DONE) {
            switch (currentState) {
                case START_DELAY:
                    telemetry.addData("State", "1. Start Delay");
                    if (runtime.seconds() >= START_WAIT_SECONDS) {
                        setAllPower(FORWARD_POWER);
                        runtime.reset();
                        currentState = State.MOVING_FORWARD;
                    }
                    break;

                case MOVING_FORWARD:
                    telemetry.addData("State", "2. Moving Forward");
                    if (runtime.seconds() >= MOVE_FORWARD_SECONDS) {
                        setAllPower(0);
                        runtime.reset();
                        currentState = State.STOPPING;
                    }
                    break;

                case STOPPING:
                    telemetry.addData("State", "3. Stopping");
                    currentState = State.DONE;
                    break;

                case DONE:
                    telemetry.addData("State", "4. Done");
                    break;
            }
            telemetry.addData("Time in State", "%.2f", runtime.seconds());
            telemetry.update();
        }

        setAllPower(0);

        while (opModeIsActive()) {
            telemetry.addData("State", "4. Done");
            telemetry.update();
        }
    }

    private void setAllPower(double power) {
        fl.setPower(power);
        rl.setPower(power);
        fr.setPower(power);
        rr.setPower(power);
    }
}
