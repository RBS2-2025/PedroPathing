package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ActionManaging_new {
    public Servo LiftServo, rotateWheel;
    public DcMotor OuttakeDc, IntakeDc;
    private ElapsedTime timer;
    private LinearOpMode opMode;

    private static final double INITIAL_POSITION = 0.5;
    private static final double ROTATION_STEP = 0.0772;

    private double currentWheelPosition = INITIAL_POSITION;

    private static final double MAX_LIMIT = 0.85;
    private static final double MIN_LIMIT = 0.15;


    public ActionManaging_new(Servo LiftServo, LinearOpMode opMode, Servo rotateWheel) {
        this.LiftServo = LiftServo;
        this.opMode = opMode;
        this.timer = new ElapsedTime();
        this.rotateWheel = rotateWheel;
        //this.IntakeDc = intakeDc;

        resetRotateWheelPosition();
    }

    public void operateLift(double duration) {
        LiftServo.setPosition(1);
        sleepFor(duration);
        LiftServo.setPosition(0);
    }

    public void sleepFor(double seconds) {
        timer.reset();
        while (opMode.opModeIsActive() && timer.seconds() <= seconds) {
            // 대기
        }
        timer.reset();
    }


    /**
     * 현재 위치를 기준으로 step만큼 이동합니다.
     * 예: steps가 1이면 +0.0772, 2면 +0.1544, -1이면 -0.0772
     */
    public void moveWheelRelatively(int steps) {
        // 현재 위치에 스텝만큼 값을 더함
        currentWheelPosition += (steps * ROTATION_STEP);

         if (currentWheelPosition > 1.0) currentWheelPosition = 1.0;
         if (currentWheelPosition < 0.0) currentWheelPosition = 0.0;

        rotateWheel.setPosition(currentWheelPosition);
        sleepFor(0.3); // 서보 이동 시간 대기
    }

    /**
     * 특정 위치로 바로 이동 (절대값)
     * 이동 후 현재 위치 변수(currentWheelPosition)도 업데이트
     */
    public void rotateToArtifactPosition(double value) {
        currentWheelPosition = value;
        rotateWheel.setPosition(currentWheelPosition);
        sleepFor(0.3);
    }

    /**
     * 초기 위치(0.5)로 리셋
     */
    public void resetRotateWheelPosition() {
        currentWheelPosition = INITIAL_POSITION;
        rotateWheel.setPosition(currentWheelPosition);
        sleepFor(0.3);
    }


    public double getCurrentWheelPosition() {
        return currentWheelPosition;
    }

    public boolean needsReset() {
        return currentWheelPosition >= MAX_LIMIT || currentWheelPosition <= MIN_LIMIT;
    }

    public boolean isNotCentered() {
        return Math.abs(currentWheelPosition - INITIAL_POSITION) > 0.01;
    }
}