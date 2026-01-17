package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Movement.IMU_Driving;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;



public class vision {

    public Limelight3A limelight;
    public List<Integer> colorArray = new ArrayList<>(Arrays.asList(0, 0, 0)); // 초록 1 보라 0
    public Telemetry telemetry;

    public IMU_Driving drive;

    public double tx ;

    public double cam_height = 0; // 카메라 높이
    public double target_height = 0; // 골대 태그 높이
    public double cam_angle = 0; // 카메라 설치 각도


    public void VisionModule(HardwareMap hardwareMap, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // 0번 파이프라인 (예: AprilTag)
        limelight.start();
    }
    public void stop() {

        limelight.stop();
    }

    public void decode(){
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) { // 인식된 태그마다 출력되게 반복
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());

                // --- 모티프 aprilTag --- //
                if (fr.getFiducialId() == 21) {
                    telemetry.addData("arr", "GPP");
                    colorArray = Arrays.asList(1, 0, 0);
                }
                if (fr.getFiducialId() == 22) {
                    telemetry.addData("arr", "PGP");
                    colorArray = Arrays.asList(0, 1, 0);
                }
                if (fr.getFiducialId() == 23) {
                    telemetry.addData("arr", "PPG");
                    colorArray = Arrays.asList(0, 0, 1);
                }
            }
        }
    }

    public void align(DcMotor dc, boolean Blue, double Kp, double maxPower, double deadzone ) {
        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) {
            dc.setPower(0);
            return;
        }
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            dc.setPower(0);
            return;
        }
        int targetId = Blue ? 20 : 24;
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() != targetId) continue;
            tx = fr.getTargetXDegrees();
            double power = tx * Kp;
            if (power > maxPower) power = -maxPower;
            if (power < -maxPower) power = maxPower;
            if (Math.abs(tx) < deadzone) power = 0;
            dc.setPower(power);
            return;
        }
    }

    public void scan(){
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            telemetry.addData("tx",tx);
            telemetry.addData("ty",ty);
            telemetry.addData("ta",ta);
            telemetry.addData("Dist",getDistanceFromTage(ta));
        }else{
            tx = 0;
        }
    }

    public double getDistanceFromTage(double ta){
        double scale = 30665.95;
        return (scale / ta);
    }
    // ^ 실패

    public double getDistanceFromTy() {
        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) return -1;

        double ty = result.getTy();

        double angleDeg = cam_angle + ty;
        double angleRad = Math.toRadians(angleDeg);

        if (Math.abs(angleDeg) < 1e-3) return -1;

        return (target_height - cam_height) / Math.tan(angleRad);
    }

    public boolean alignAuto(DcMotor turret, boolean Blue, double speed) {

        LLResult result = limelight.getLatestResult();

        // 1) 프레임이 유효하지 않으면 멈추고 계속 반복(정렬 미완료)
        if (result == null || !result.isValid()) {
            turret.setPower(Blue? 0.05:-0.05);
            return false;
        }

        // 2) Fiducial(result) 리스트 가져오기
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            turret.setPower(0);
            return false;
        }

        // 3) 목표 골 ID — 기존 align()과 동일
        int targetId = Blue ? 20 : 24;

        // 4) 탐색
        for (LLResultTypes.FiducialResult fr : fiducialResults) {

            if (fr.getFiducialId() != targetId) continue;

            // tx = 가로 각도 오프셋(기존 align() 코드 기반)
            double tx = fr.getTargetXDegrees();
            double deadband = 2.0; // 정렬 완료 기준 오차 (조금 더 민감하게 2도 추천)

            // 4-1) 정렬 완료 조건
            if (Math.abs(tx) <= deadband) {
                turret.setPower(0);
                return true;   // ★★★ Action 종료됨
            }

            // 4-2) 아직 정렬 안 됨 → 계속 회전
            if (tx > 0) {
                turret.setPower(-speed); // 오른쪽에 목표 → 왼쪽으로 회전
            } else {
                turret.setPower(speed);  // 왼쪽에 목표 → 오른쪽으로 회전
            }

            return false;  // 아직 정렬 중
        }

        // 5) targetId 못 찾음 → 정지, 계속 탐색
        turret.setPower(0);
        return false;
    }

    public void setHood(Servo hood){
        double dist = getDistanceFromTy();
        if (dist < 0) return;

        double pos;

        if (dist < 150) {          // 가까움
            pos = 0.20;
        } else if (dist < 200) {   // 중간
            pos = 0.30;
        } else {                   // 멂
            pos = 0.40;
        }

        hood.setPosition(pos);

        telemetry.addData("Dist", dist);
        telemetry.addData("Hood", pos);

    }




}