package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ActionManaging {
    public Servo lifting, wheel;
    public DcMotor OuttakeDc, IntakeDc;
    private ElapsedTime timer;
    private LinearOpMode opMode;

    // ----- New fields for RotateWheel management -----
    private static final double[] ARTIFACT_POSITIONS = {0.0, 0.4, 0.8}; // 구조화된 값 (RotateWheel 위치)
    private int currentRotatePositionIndex = 0; // RotateWheel의 현재 위치 인덱스

    public ActionManaging(DcMotor IntakeDc, DcMotor OuttakeDc, Servo lift, LinearOpMode opMode, Servo wheel) {
        this.OuttakeDc = OuttakeDc;
        this.IntakeDc = IntakeDc;
        this.lifting = lift;
        this.opMode = opMode;
        this.timer = new ElapsedTime();
        this.wheel = wheel;

        // RotateWheel을 초기 위치로 설정
        wheel.setPosition(ARTIFACT_POSITIONS[0]);
        currentRotatePositionIndex = 0;
    }

    public void intake(double power_in) {
        IntakeDc.setPower(power_in);
    }

    public void intake_stop() {
        IntakeDc.setPower(0);
    }

    public void intake_r() {
        IntakeDc.setPower(-1);
    }

    // New: operateLift with duration
    public void operateLift(double duration) {
        lifting.setPosition(1);
        sleepFor(duration);
        lifting.setPosition(0);
    }

    public void outtake(double power) {
        OuttakeDc.setPower(power);
    }

    public void outtake_stop() {
        OuttakeDc.setPower(0);
        IntakeDc.setPower(0);
    }

    public void sleepFor(double seconds) {
        timer.reset();
        while (opMode.opModeIsActive() && timer.seconds() <= seconds) {
            // Nothing
        }
        timer.reset();
    }

    // ----- New methods for RotateWheel control -----

    /**
     * Resets the RotateWheel to the first artifact position (index 0).
     */
    public void resetRotateWheelPosition() {
        wheel.setPosition(ARTIFACT_POSITIONS[0]);
        currentRotatePositionIndex = 0;
        sleepFor(0.3); // 서보 이동 시간 대기
    }

    /**
     * Advances the RotateWheel to the next artifact position in the sequence.
     * Cycles back to the first position after the last.
     */
    public void advanceRotateWheelPosition() {
        currentRotatePositionIndex = (currentRotatePositionIndex + 1) % ARTIFACT_POSITIONS.length;
        wheel.setPosition(ARTIFACT_POSITIONS[currentRotatePositionIndex]);
        sleepFor(0.3); // 서보 이동 시간 대기
        opMode.telemetry.addData("RotateWheel", "Moved to: %.1f", ARTIFACT_POSITIONS[currentRotatePositionIndex]);
        opMode.telemetry.addData("Next Scan Index", currentRotatePositionIndex);
        opMode.telemetry.update();
    }

    /**
     * Moves the RotateWheel directly to a specific artifact position by index.
     * @param index The index of the target position in ARTIFACT_POSITIONS.
     */
    public void rotateToArtifactPosition(double value) {
        wheel.setPosition(value);
        sleepFor(0.3); // 서보 이동 시간 대기
        opMode.telemetry.addData("RotateWheel", "Moved to: %.1d", value);
        opMode.telemetry.update();
    }

    /**
     * Stops all motors and resets servos to their initial positions.
     */
    public void stopAll() {
        IntakeDc.setPower(0);
        OuttakeDc.setPower(0);
        lifting.setPosition(0); // 리프트 내림
        wheel.setPosition(ARTIFACT_POSITIONS[0]); // Rotate wheel을 초기 위치로
        currentRotatePositionIndex = 0;
    }

    /**
     * Returns the current index of the RotateWheel's position.
     */
    public int getCurrentRotatePositionIndex() {
        return currentRotatePositionIndex;
    }
}