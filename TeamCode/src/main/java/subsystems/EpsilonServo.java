package subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EpsilonServo {
    private Servo outakeflipperR;
    private Servo outakeflipperL;
    private Servo pin;
    private DcMotor ShooterL;
    private DcMotor ShooterR;
    private ElapsedTime stateTimer0 = new ElapsedTime();
    private enum ShootState{
        Init,
        Idle,
        Spin_Up,
        ShootL,
        ShootR,
        Reset
    }
    private ShootState shootState;
    // FlipperConstants
    private double FlipLClose = 0;
    private double FlipLOpen = 0;
    private double FlipRClose = 0;
    private double FlipROpen = 0;
    private double FlipperTimeUp = 0.25;
    private double FlipperTimeDown = 0.25;
    // ShootConstants
    private int shotsRemaining = 0;
    private double velo = 0;
    //private double RPMmin = 800; //Probably won't use rpm in auto since voltage should be consistent
    //private double RPMtarget = 1200;
    private double shootPower = .45;
    private void init(HardwareMap hwMap){
        outakeflipperR = hwMap.get(Servo.class,"outakeflipperR");
        outakeflipperL = hwMap.get(Servo.class,"outakeflipperL");
        pin = hwMap.get(Servo.class,"pin");
        ShooterL = hwMap.get(DcMotor.class,"ShooterL");
        ShooterR = hwMap.get(DcMotor.class,"ShooterR");
        ShooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Tune PIDF
        shootState = ShootState.Init;
        outakeflipperL.setPosition(FlipLClose);
        outakeflipperR.setPosition(FlipRClose);
        ShooterL.setPower(0);
        ShooterR.setPower(0);
    }
    public void update(){
        switch (shootState) {
            case Init:
                outakeflipperL.setPosition(FlipLClose);
                outakeflipperR.setPosition(FlipRClose);
                ShooterL.setPower(0);
                ShooterR.setPower(0);
                stateTimer0.reset();
                shootState = ShootState.Spin_Up;

                break;
            case Spin_Up:
        }
    }
}
