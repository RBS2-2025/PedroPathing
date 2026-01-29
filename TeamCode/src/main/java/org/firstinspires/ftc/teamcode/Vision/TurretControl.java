package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.utils.PanelsHelper;

import java.util.List;

public class TurretControl {
    public Limelight3A limelight;
    public DcMotorEx turretMotor;

    public double tx; //x 차이

    public double cam_height = 0; // 카메라 높이 : MM
    public double target_height = 754; // 골대 태그 높이 : MM
    public double cam_angle = 0; // 카메라 설치 각도
    public boolean isBlue;
    public PanelsHelper panel;
    int targetID;
    public double deadZone = 0.2;
    IMU imu;

    public TurretControl(Robot robot, boolean blue, IMU imu){
//        this.limelight = hardwareMap.get(Limelight3A.class,"limelight");
        this.limelight = robot.limelight;
        this.turretMotor = robot.tracker;

        this.isBlue = blue;
        this.targetID = this.isBlue? 20 : 24;
//        this.targetID = 23;
        this.imu = imu;


        limelight.pipelineSwitch(0); // 0번 파이프라인 (예: AprilTag)
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }


    public void align(double maxPower, boolean byPixel, double speed){
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
        LLResult result = limelight.getLatestResult();
        if(!result.isValid()) {
//            panel.addData("status","not valid");
            if(Math.abs(turretMotor.getCurrentPosition()) < 20){
                turretMotor.setPower(0);
                return;
            }
            turretMotor.setTargetPosition(0);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(maxPower);
            return;
        }
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if(fiducialResults.isEmpty()) {
            turretMotor.setPower(0);
//            panel.addData("status","not found");
            return;
        }
        for(LLResultTypes.FiducialResult fr : fiducialResults){
            if (fr.getFiducialId() != targetID) continue;
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tx = fr.getTargetXDegrees() * speed;
//            panel.addData("tx",tx);
            if(Math.abs(tx) < deadZone) {
                turretMotor.setPower(0);
                return;
            }
            double power = -tx;

            if(Math.abs(power) > maxPower) {
                power = Math.signum(power) * maxPower;
            }
//            panel.addData("power",power);
            if (turretMotor.getCurrentPosition() > 300) {
                turretMotor.setPower(Math.min(0, power)); // +방향 차단
                return;
            }
            if (turretMotor.getCurrentPosition() < -300) {
                turretMotor.setPower(Math.max(0, power)); // -방향 차단
                return;
            }
            turretMotor.setPower(power);
            return;
        }
        if(Math.abs(turretMotor.getCurrentPosition()) < 2){
            turretMotor.setPower(0);
            return;
        }
//        panel.addData("status","skipped");

        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(maxPower);

        return;
    }

    public double getDistance(){
        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) return -1;
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if(fiducialResults.isEmpty()) return -1;

        for (LLResultTypes.FiducialResult fr: fiducialResults){
            if(fr.getFiducialId() != targetID) continue;
            double angle = Math.toRadians(cam_angle + fr.getTargetYDegrees());
            if (Math.abs(Math.toDegrees(angle)) < 1e-3) return -1;
            return (target_height-cam_height) / Math.tan(angle);
        }
        return -1;
    }
}
