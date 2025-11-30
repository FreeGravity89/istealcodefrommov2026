package org.firstinspires.ftc.teamcode.autonomous;
import static com.qualcomm.hardware.limelightvision.LLResultTypes.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;


public class LimelightTestes extends OpMode {
    public Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        limelight.start(); // This tells Limelight to start looking!
    }
    @Override
    public void start(){

    }

    @Override
    public void loop() {
        //LLResult result = limelight.getLatestResult();
       // if (result != null && result.isValid()) {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            //telemetry.addData("ID", llResult.get());
            List<FiducialResult> fiducials = llResult.getFiducialResults();
            for (FiducialResult fiducialResult : fiducials) {
                int id = fiducialResult.getFiducialId();
                telemetry.addData("IDTAG", id);
            }
        }

    }
}
