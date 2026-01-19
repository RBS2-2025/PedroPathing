package org.firstinspires.ftc.teamcode.Teleop;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Movement.ActionManaging;
// Webcam 1
@Configurable
@TeleOp
public class New_Sorting extends LinearOpMode {

    // 하드웨어 (센서는 색상을 감지해야 하므로 로컬 필요)
    private NormalizedColorSensor c1, c2, c3;

    // ActionManaging 객체
    private ActionManaging ActionManager;
    public Servo rotateWheel,LiftServo;

    // 색 기준값
    public static final float[] GREEN = new float[]{0.20f, 0.55f, 0.25f};
    public static final float[] PURPLE = new float[]{0.40f, 0.25f, 0.45f};

    // 파라미터
    private static final float COLOR_THRESHOLD = 0.20f; // ±20% 범위
    public static final double ROTATE_DURATION_LIFT_SERVO = 0.5; // 초
    private static final double ROTATE_TIME = 1.0;

    // 무한 솔팅 제어 변수
    private boolean isInfiniteSortingActive = false;
    private int colorSortIndex = 0;
    public double current_position = 0.5;
    public static double test1 = 0.5-0.0772;
    public static double test2 = 0.5;
    public static double test3 = 0.5772;
    public static double step = 0.0772;
    final float[] hsv1 = new float[3];
    final float[] hsv2 = new float[3];
    final float[] hsv3 = new float[3];
    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // gamepad1.y를 누르면 무한 솔팅 시작
            if (gamepad1.y && !isInfiniteSortingActive) {
                isInfiniteSortingActive = true;
                telemetry.addData("Sorting Mode", "Infinite Sorting - Active");
                telemetry.update();
            }
            // gamepad1.x를 누르면 무한 솔팅 중지
            if (gamepad1.x && isInfiniteSortingActive) {
                isInfiniteSortingActive = false;
                telemetry.addData("Sorting Mode", "Infinite Sorting - Stopped");
                telemetry.update();
                ActionManager.stopAll();
            }

            if (isInfiniteSortingActive) {
                // 초록색과 보라색을 번갈아 가며 솔팅 (실제 Infinite Sorting 로직)
                String targetColor;
                if (colorSortIndex == 0) {
                    targetColor = "Green";
                } else {
                    targetColor = "Purple";
                }

                // detectColor는 센서 값을 읽어야 하므로 여기 유지
                // sortForColor 내부에서 ActionManager를 사용하도록 수정됨
                sortForColor(targetColor);

                colorSortIndex = (colorSortIndex + 1) % 2; // 인덱스 순환

                // 솔팅 주기 조절을 위한 대기
                ActionManager.sleepFor(0.2);

            } else {
                // 수동 솔팅 로직
                if (gamepad1.a) {
                    sortForColor("Green");
                    ActionManager.sleepFor(0.5);
                } else if (gamepad1.b) {
                    sortForColor("Purple");
                    ActionManager.sleepFor(0.5);
                } else if (gamepad1.dpad_left) {
                    ActionManager.rotateToArtifactPosition(test1);
                } else if (gamepad1.dpad_down) {
                    ActionManager.rotateToArtifactPosition(test2);
                } else if (gamepad1.dpad_right) {
                    ActionManager.rotateToArtifactPosition(test3);
                }
            }

            NormalizedRGBA c1c = c1.getNormalizedColors();
            Color.colorToHSV(c1c.toColor(), hsv1);

            NormalizedRGBA c2c = c2.getNormalizedColors();
            Color.colorToHSV(c2c.toColor(), hsv2);

            NormalizedRGBA c3c = c3.getNormalizedColors();
            Color.colorToHSV(c3c.toColor(), hsv3);

            // 120 None  140 G 180 P
            // 0 None  1 G 0.5 P
            telemetry.addData("C1", detectColorHSV(c1));
            telemetry.addData("C2", detectColorHSV(c2));
            telemetry.addData("C3", detectColorHSV(c3));
            telemetry.addData("C1 h", hsv1[0]);
            telemetry.addData("C1 s", hsv1[1]);
            telemetry.addData("C1 v", hsv1[2]);

            telemetry.addData("C2 h", hsv2[0]);
            telemetry.addData("C2 s", hsv2[1]);
            telemetry.addData("C2 v", hsv2[2]);

            telemetry.addData("C3 h", hsv3[0]);
            telemetry.addData("C3 s", hsv3[1]);
            telemetry.addData("C3 v", hsv3[2]);
            // 상태 표시 (모터 파워 등은 ActionManager getter가 필요할 수 있으나, 여기선 단순 상태 표시만)
            telemetry.addData("Infinite Sorting", isInfiniteSortingActive ? "ACTIVE" : "INACTIVE");

            telemetry.update();
        }
    }
// green saturation 0.7
    // purple hue 200
    // ------------------------
// 하드웨어 초기화
// ------------------------
    public void initHardware() {
        // 하드웨어 맵핑
        rotateWheel = hardwareMap.get(Servo.class, "RotateWheel");
//        DcMotor intakeWheel = hardwareMap.get(DcMotor.class, "IntakeWheel");
//        DcMotor outtakeWheel = hardwareMap.get(DcMotor.class, "OuttakeWheel");
        LiftServo = hardwareMap.get(Servo.class, "LiftServo");

        c1 = hardwareMap.get(NormalizedColorSensor.class, "c1");
        c2 = hardwareMap.get(NormalizedColorSensor.class, "c2");
        c3 = hardwareMap.get(NormalizedColorSensor.class, "c3");
        c1.setGain(8);
        c2.setGain(8);
        c3.setGain(8);
        // ActionManaging 초기화
        ActionManager = new ActionManaging(LiftServo, this, rotateWheel);

        // 하드웨어 설정 (방향 등)
//        intakeWheel.setDirection(DcMotor.Direction.FORWARD);
//        outtakeWheel.setDirection(DcMotor.Direction.REVERSE);
        rotateWheel.setDirection(Servo.Direction.FORWARD);
        LiftServo.setDirection(Servo.Direction.FORWARD);

        // 초기 위치 설정
        rotateWheel.setPosition(0.5);
        LiftServo.setPosition(0.0);

//        intakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        outtakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ------------------------
// 분류 로직: 우선순위 c1 (Lift) -> c2 -> c3
// 센싱 최적화: 필요한 센서만 순차적으로 읽어서 연산 및 I/O 최소화
// ------------------------
    private void sortForColor(String targetColor) {
        telemetry.addData("Target Color", targetColor);
        String resultAction = "Scanning...";

        // 1순위: Lift 위치(c1) 확인
        if (isTargetColor(c1, targetColor)) {
            telemetry.addLine("Action: Lift Item from c1");
            ActionManager.operateLift(ROTATE_DURATION_LIFT_SERVO);
            resultAction = "Lifted from c1";
        }
        // 2순위: 중간 위치(c2) 확인 (c1이 아닐 경우에만 센싱)
        else if (isTargetColor(c2, targetColor)) {
            telemetry.addLine("Action: Rotate 1 & Lift Item from c2");
            // 1칸 이동(Next)과 같지만, 의미적으로 "1단계 회전"으로 명시
            ActionManager.advanceRotateWheelPosition();

            // 안정화 시간은 rotateBySteps 내부에 포함됨 (0.3s)
            ActionManager.operateLift(ROTATE_DURATION_LIFT_SERVO);
            resultAction = "Lifted from c2";
        }
        // 3순위: 먼 위치(c3) 확인 (c1, c2가 아닐 경우에만 센싱)
        else if (isTargetColor(c3, targetColor)) {
            telemetry.addLine("Action: Rotate 2 & Lift Item from c3");
            // 2칸 이동 (중간 단계를 거치지 않고 바로 목표 위치로 이동 -> 시간 절약)
            ActionManager.advanceRotateWheelPosition();
            ActionManager.advanceRotateWheelPosition();

            // 안정화 시간은 rotateBySteps 내부에 포함됨 (0.3s)
            ActionManager.operateLift(ROTATE_DURATION_LIFT_SERVO);
            resultAction = "Lifted from c3";
        } else {
            // 아무것도 감지되지 않음
            resultAction = "None Found";
        }

        telemetry.addData("Result", resultAction);
    }

    // 색상 감지 여부를 판단하는 최적화된 메서드
    private boolean isTargetColor(NormalizedColorSensor sensor, String targetColor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();

        // HSV 변환
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);

        if (targetColor.equals("Green")) {
            // 초록색: Saturation >= 0.7
            return hsv[1] >= 0.6;
        } else if (targetColor.equals("Purple")) {
            // 보라색: Hue >= 200
            return hsv[0] >= 180;
        }

        return false;
    }
    private static final float[] GREEN_old = new float[]{0.20f, 0.55f, 0.25f};
    private static final float[] PURPLE_old = new float[]{0.40f, 0.25f, 0.45f};
    private boolean isTargetColor_dld(NormalizedColorSensor sensor, String targetColor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float[] targetRGB = targetColor.equals("Green") ? GREEN : PURPLE;

        // 미리 계산된 범위 내에 있는지 확인 (연산 최소화)
        // Red check
        if (colors.red < targetRGB[0] * (1 - COLOR_THRESHOLD) || colors.red > targetRGB[0] * (1 + COLOR_THRESHOLD))
            return false;
        // Green check
        if (colors.green < targetRGB[1] * (1 - COLOR_THRESHOLD) || colors.green > targetRGB[1] * (1 + COLOR_THRESHOLD))
            return false;
        // Blue check
        if (colors.blue < targetRGB[2] * (1 - COLOR_THRESHOLD) || colors.blue >   targetRGB[2] * (1 + COLOR_THRESHOLD))
            return false;

        return true;
    }

    private String detectColorHSV(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();

        float[] hsv = new float[3];
        Color.colorToHSV(rgba.toColor(), hsv);

        float hue = hsv[0];        // 0 ~ 360
        float saturation = hsv[1]; // 0 ~ 1

        if (saturation >= 0.6) {
            return "green";
        }

        if (hue >= 180) {
            return "purple";
        }

        return "none";
    }


}