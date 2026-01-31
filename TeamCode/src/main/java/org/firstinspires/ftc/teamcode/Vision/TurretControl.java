package org.firstinspires.ftc.teamcode.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.utils.PanelsHelper;

import java.util.List;

public class TurretControl {
    private final Follower follower;
    private final DcMotorEx turretMotor;
    private final Robot robot;

    // !!!! WARNING: 튜닝 필요 !!!!
    public static double TICKS_PER_DEGREE = (double) 10 / 3;

    // 골대 위치
    public static final Pose BLUE_BASKET = new Pose(0, 144);
    public static final Pose RED_BASKET = new Pose(144, 144);

    private final Pose targetGoalPose;
    private final int targetID;

    // ★ 튜닝 포인트 1: 비전 데드존 (단위: 도)
    // tx 오차가 이 값보다 작으면 보정하지 않음 -> 진동 방지 핵심
    public double visionDeadZone = 1.0;

    // ★ 튜닝 포인트 2: 비전 반영 속도 (Gain)
    // 0.02는 너무 느릴 수 있고, 너무 크면 진동함. 적당한 값 찾기.
    public double visionGain = 0.05;

    private double visionOffsetDeg = 0;

    // 회전 제한
    double MAX_ROTATION_DEG = 90;

    public TurretControl(Robot robot, Follower follower, boolean isBlue){
        this.robot = robot;
        this.turretMotor = robot.tracker;
        this.follower = follower;

        follower.update();
        this.targetGoalPose = isBlue ? BLUE_BASKET : RED_BASKET;
        this.targetID = isBlue ? 20 : 24; // Blue 20, Red 24 (확인 필요)

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.3); // RUN_TO_POSITION에서는 파워를 1.0(최대)으로 두고 내부 PID에 맡기는 게 가장 떱니다.
    }

    public void update() {
        Pose robotPose = follower.getPose();

        // 1. 기본 오도메트리 각도 계산
        double dx = targetGoalPose.getX() - robotPose.getX();
        double dy = targetGoalPose.getY() - robotPose.getY();
        double angleToGoalRad = Math.atan2(dy, dx);

        double robotHeadingRad = robotPose.getHeading();
        double relativeAngleRad = angleToGoalRad - robotHeadingRad;

        // 2. 각도 정규화
        relativeAngleRad = normalizeAngle(relativeAngleRad);
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);

        // 3. 비전 보정 업데이트 (여기서 오프셋 변경)
        updateVisionCorrection();

        // 4. 최종 목표 각도
        double finalTargetDeg = relativeAngleDeg + visionOffsetDeg;

        // 5. 범위 제한 (90도 넘어가면 정면 보기)
        if(Math.abs(finalTargetDeg) > MAX_ROTATION_DEG){
            finalTargetDeg = 0;
            // 범위 밖일 때 오프셋을 초기화할지 유지할지는 선택 (보통 유지가 나음)
            // visionOffsetDeg = 0;
        }


        // 6. 모터 명령
        int targetTicks = (int) (finalTargetDeg * TICKS_PER_DEGREE);

        // [중요 수정] RUN_TO_POSITION 사용 시, 파워는 그냥 1.0(Max)으로 고정하세요.
        // 거리 비례해서 파워를 줄이면, 목표지점 근처에서 힘이 없어서 웅웅거리거나(Deadband) 밀립니다.
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.35);
    }

    private double normalizeAngle(double angleRadians) {
        while (angleRadians > Math.PI) angleRadians -= 2 * Math.PI;
        while (angleRadians < -Math.PI) angleRadians += 2 * Math.PI;
        return angleRadians;
    }
    public void getDebug(PanelsHelper panels){
        Pose robotPose = follower.getPose();
        double dx = targetGoalPose.getX() - robotPose.getX();
        double dy = targetGoalPose.getY() - robotPose.getY();
        double angleToGoalRad = Math.atan2(dy, dx);

        double robotHeadingRad = robotPose.getHeading();
        double relativeAngleRad = angleToGoalRad - robotHeadingRad;
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);
        double finalTargetDeg = relativeAngleDeg + visionOffsetDeg;
        int targetTicks = (int) (finalTargetDeg * TICKS_PER_DEGREE);
        panels.addData("dx",dx);
        panels.addData("dy",dy);
        panels.addData("angleToGoalRad",angleToGoalRad);
        panels.addData("heading",angleToGoalRad);
        panels.addData("relativeAngle",relativeAngleDeg);
        panels.addData("finalTarget",finalTargetDeg);
        panels.addData("target Tick",targetTicks);

    }

    private void updateVisionCorrection() {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                // ID 필터링 (엉뚱한 태그 보고 돌지 않게)
                if(tag.getFiducialId() != targetID) continue;

                double tx = tag.getTargetXDegrees();

                // ★ [핵심] 비전 데드존 적용
                // 오차(tx)가 너무 작으면(이미 잘 맞으면) 보정하지 않음 -> 진동 멈춤
                if (Math.abs(tx) < visionDeadZone) {
                    return;
                }

                // 적분 제어 (누적)
                // tx가 +면 오른쪽으로 더 돌아야 함 -> 오프셋 증가
                // tx가 -면 왼쪽으로 더 돌아야 함 -> 오프셋 감소
                double correction = tx * visionGain;

                // 과도한 누적 방지 (Limit)
                // 오프셋이 너무 커지면(예: 20도 이상) 오도메트리가 완전히 틀렸다는 뜻이므로 위험함.
                if (Math.abs(visionOffsetDeg + correction) < 15.0) {
                    visionOffsetDeg += correction;
                }

                // 태그 하나만 보고 루프 종료 (여러 개 보일 때 중복 적용 방지)
                return;
            }
        }
    }
}