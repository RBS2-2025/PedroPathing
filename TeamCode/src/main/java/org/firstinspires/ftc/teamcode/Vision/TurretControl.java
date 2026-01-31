package org.firstinspires.ftc.teamcode.Vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import java.util.List;

public class TurretControl {
    private final Follower follower;
    private final DcMotorEx turretMotor;
    private final Robot robot;

    // !!!! WARNING START: need tuning !!!!
    public static double TICKS_PER_DEGREE = (double) 10 /3;
    // !!!! WARNING END: need tuning !!!!
    double MAX_ROTATION_DEG = 90;
    // 골대 위치
    public static final Pose BLUE_BASKET = new Pose(0, 144);
    public static final Pose RED_BASKET = new Pose(144, 144);

    private final Pose targetGoalPose;
    int targetID;

    // 라임라이트 보정용 변수
    private double visionOffsetDeg = 0;
    public double deadZone = 5;

    public TurretControl(Robot robot, Follower follower, boolean isBlue){
        this.robot = robot;
        this.turretMotor = robot.tracker;
        this.follower = follower;

        this.targetGoalPose = isBlue ? BLUE_BASKET : RED_BASKET;
        this.targetID = isBlue? 20: 24;

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update() {
        Pose robotPose = follower.getPose();

        double dx = targetGoalPose.getX() - robotPose.getX();
        double dy = targetGoalPose.getY() - robotPose.getY();
        double angleToGoalRad = Math.atan2(dy, dx); // 아크탄젠트로 목표각도 계산

        double robotHeadingRad = robotPose.getHeading(); // 로봇 현재각도 get
        double relativeAngleRad = angleToGoalRad - robotHeadingRad; // 목표각도 - 현재각도 = 회전해야할 각도

        // 각도 정규화 (회전해야할 각도는 정규화 (-pi ~ pi)되지 않았을 수 있음)
        relativeAngleRad = normalizeAngle(relativeAngleRad);
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);

        updateVisionCorrection(); // 라임라이트 보정



        double finalTargetDeg = relativeAngleDeg + visionOffsetDeg; // 최종 보정값

        if(Math.abs(finalTargetDeg) > MAX_ROTATION_DEG){
            finalTargetDeg = 0;
            visionOffsetDeg = 0;
        }
        if(Math.abs(relativeAngleDeg) < deadZone){
            turretMotor.setPower(0);
            return;
        }

        int targetTicks = (int) (finalTargetDeg * TICKS_PER_DEGREE); // 모터 명령부

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.3 * (Math.max(relativeAngleDeg/90,0.06)));
    }

    // 각도 정규화 함수 (어디 찾으면 내장함수로 있을 건데 못 찾아서 그냥 직접 씀)
    private double normalizeAngle(double angleRadians) {
        while (angleRadians > Math.PI) angleRadians -= 2 * Math.PI;
        while (angleRadians < -Math.PI) angleRadians += 2 * Math.PI;
        return angleRadians;
    }

    private void updateVisionCorrection() {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                // 필요하다면 여기서 태그 ID 필터링 (Blue: 20, Red: 24 등 -> 굳이?)
                if(tag.getFiducialId() != targetID) return;
                double tx = tag.getTargetXDegrees();
                double power = tx * 0.02;
                power = Math.min(3.5,Math.max(-3.5,power));
                visionOffsetDeg += power; // 게인값 조절 (너무 빠르면 줄이기)
            }
        }
    }
}