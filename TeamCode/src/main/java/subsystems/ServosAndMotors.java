package subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class ServosAndMotors {
    public Limelight3A limelight;
    private Servo trapdoory;
    private Servo outakeflipperL;
    private Servo pin;
    private DcMotor ShooterL;
    private DcMotor ShooterR;
    private ElapsedTime stateTimer0 = new ElapsedTime();
    private enum ShootState{
        AprilTag,
        IdlePPG,
        IdlePGP,
        IdleGPP,
        Spin_UpPPG,
        Spin_upPGP,
        Spin_upGPP,
        PPG1,
        PPG2,
        PPG3,
        PGP1,
        PGP2,
        GPP1,
        GPP2,
        GPP3,
        green,
        none,
        Intake1and2,
        Intake3,
        Reset
    }
    private ShootState shootState;
   // public int Apriltag = 21;//replace once apriltag detection works
    // FlipperConstants
   public int Apriltag = 0;//replace once apriltag detection works
    private double FlipLClose =.715;
    private double FlipLOpen =.5;
    private double FlipRClose =.26;
    private double FlipROpen =.6;
    private double FlipperTimePurpReset = 0.15;//how long before Flipper does a flippy
    private double FlipperTimeGreeReset =0.15;
    private double FlipperTimePurp = 0.25;
    private double FlipperTimeGree = 0.25;
    private double shootTime1 = .5; //This is adjusing to how long before it shoots
    // ShootConstants
    public int shotsRemaining = 1;
    private double velo = 0;
    //private double RPMmin = 800; //Probably won't use rpm in auto since voltage should be consistent
    //private double RPMtarget = 1200;
    private double shootPower = 0;

    public void init(HardwareMap hwMap){
        trapdoory = hwMap.get(Servo.class,"trapdoory");
        outakeflipperL = hwMap.get(Servo.class,"outakeflipL");
        pin = hwMap.get(Servo.class,"pin");
        ShooterL = hwMap.get(DcMotor.class,"outakeL");
        ShooterR = hwMap.get(DcMotor.class,"outakeR");
        ShooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ShooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Limelight ----------------------------------------------------------
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        limelight.start(); // This tells Limelight to start looking!
        stateTimer0.reset();
        //Tune PIDF, Maybe...
        shootState = ShootState.IdleGPP;
        outakeflipperL.setPosition(FlipLClose);
        trapdoory.setPosition(FlipRClose);
        pin.setDirection(Servo.Direction.REVERSE);

    }
    public void update(){
        switch (shootState) {
            case AprilTag:
                if(Apriltag == 21){//fix once apriltag worky
                    stateTimer0.reset();
                    shootState = ShootState.IdleGPP;
                }
                else if (Apriltag == 22) {
                    stateTimer0.reset();
                    shootState = ShootState.IdlePGP;
                }
                else if (Apriltag == 23) {
                    stateTimer0.reset();
                    shootState = ShootState.IdlePPG;
                }
                else if (stateTimer0.seconds() > 1){//this is if it does not detect
                    stateTimer0.reset();
                    shootState = ShootState.IdlePPG;
                }
                break;
            case IdlePPG:
                outakeflipperL.setPosition(FlipLClose);
                trapdoory.setPosition(FlipRClose);
                if(shotsRemaining >0) {
                    ShooterL.setPower(shootPower);
                    ShooterR.setPower(shootPower);
                    telemetry.addLine("IdlePPG");
                    stateTimer0.reset();
                    shootState = ShootState.Spin_UpPPG;
                }
                break;
            case Spin_UpPPG:
                if(stateTimer0.seconds() > shootTime1){
                    outakeflipperL.setPosition(FlipLOpen);
                    telemetry.addLine("Spin_UpPPG");
                    stateTimer0.reset();
                    shootState = ShootState.PPG1;
                    break;
                }
            case IdlePGP:
                outakeflipperL.setPosition(FlipLClose);
                trapdoory.setPosition(FlipRClose);
                if(shotsRemaining > 0) {
                    ShooterL.setPower(shootPower);
                    ShooterR.setPower(shootPower);
                    stateTimer0.reset();
                    shootState = ShootState.Spin_upPGP;
                }
                break;
            case IdleGPP:
                outakeflipperL.setPosition(FlipLClose);
                trapdoory.setPosition(FlipRClose);
                if(shotsRemaining > 0) {
                    ShooterL.setPower(shootPower);
                    ShooterR.setPower(shootPower);
                    stateTimer0.reset();
                    shootState = ShootState.Spin_upGPP;
                }
                break;

            case Spin_upPGP:
                if(stateTimer0.seconds() > shootTime1){
                    outakeflipperL.setPosition(FlipLOpen);
                    stateTimer0.reset();
                    shootState = ShootState.PGP1;
                    break;
                }
            case Spin_upGPP:
                if(stateTimer0.seconds() > shootTime1){
                    trapdoory.setPosition(FlipROpen);
                    stateTimer0.reset();
                    shootState = ShootState.GPP1;

                    break;
                }
            case PPG1:
                if (stateTimer0.seconds() > FlipperTimePurp){
                    outakeflipperL.setPosition(FlipLClose);
                    telemetry.addLine("PPG1");
                    stateTimer0.reset();
                    shootState = ShootState.PPG2;
                    break;
                }
            case PPG2:
                if (stateTimer0.seconds() > FlipperTimePurpReset){
                    outakeflipperL.setPosition(FlipLOpen);
                    telemetry.addLine("PPG2");
                    stateTimer0.reset();
                    shootState = ShootState.PPG3;
                    break;
                }
            case PPG3:
                if (stateTimer0.seconds() > FlipperTimePurpReset){
                    outakeflipperL.setPosition(FlipROpen);
                    telemetry.addLine("PPG3");
                    stateTimer0.reset();
                    shootState = ShootState.IdlePPG;
                    break;
                }

            case PGP1:
                if (stateTimer0.seconds() > FlipperTimeGree){
                    outakeflipperL.setPosition(FlipLClose);
                    trapdoory.setPosition(FlipROpen);
                    stateTimer0.reset();
                    shootState = ShootState.PGP2;
                    break;
                }
            case PGP2:
                if (stateTimer0.seconds() > FlipperTimePurp){
                    trapdoory.setPosition(FlipRClose);
                    outakeflipperL.setPosition(FlipLOpen);
                    stateTimer0.reset();
                    shotsRemaining = 0;
                    shootState = ShootState.IdlePGP;
                    break;
                }
            case GPP1:
                if (stateTimer0.seconds() > FlipperTimePurp){
                    outakeflipperL.setPosition(FlipLOpen);
                    trapdoory.setPosition(.7);
                    stateTimer0.reset();
                    shootState = ShootState.GPP2;
                    break;
                }
            case GPP2:
                if (stateTimer0.seconds() > FlipperTimePurp){
                    outakeflipperL.setPosition(FlipLClose);
                    stateTimer0.reset();
                    shootState = ShootState.GPP3;
                    break;
                }
            case GPP3:
                if (stateTimer0.seconds() > FlipperTimePurpReset){
                    outakeflipperL.setPosition(FlipLOpen);
                    stateTimer0.reset();
                    shotsRemaining =0;
                    shootState = ShootState.IdleGPP;
                    break;
                }


        }
    }
    public void fireshots(int numberofshots){
        if(shootState == ShootState.IdlePPG || shootState == ShootState.IdlePGP || shootState == ShootState.IdleGPP){
            shotsRemaining = numberofshots;
        }
    }
    public boolean isBusy(){
        return shootState != ShootState.IdlePPG || shootState != ShootState.IdlePGP || shootState != ShootState.IdleGPP;
    }
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            //telemetry.addData("ID", llResult.get());
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducialResult : fiducials) {
                int id = fiducialResult.getFiducialId();
                //Apriltag = id;
                telemetry.addData("statetimer", stateTimer0);

            }
        }
        if (shootState != ShootState.AprilTag){
            limelight.stop();
        }
        telemetry.addData("statetimer", stateTimer0);
        telemetry.addData("shootshate",shootState.toString());
        telemetry.addData("ID",limelight);

    }
}
