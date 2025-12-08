package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "DRIVE2526 ")
public class DRIVE2526 extends LinearOpMode {

    private DcMotor outakeR;
    private DcMotor intake;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor fl;
    private DcMotor fr;
    private Servo pin;
    private Servo trapdoory;
    private Servo outakeflipL;
    private DcMotor outakeL;
    private ColorSensor color_sensor1;
    private VoltageSensor ControlHub_VoltageSensor;
    private ColorSensor color_sensor;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        double speed;
        NormalizedRGBA myNormalizedColors;
        float Blue;
        ElapsedTime GateTime;
        String Color2;
        boolean PurpleGate;
        float Green;
        float Red;
        double turbo_speed;

        outakeR = hardwareMap.get(DcMotor.class, "outakeR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        pin = hardwareMap.get(Servo.class, "pin");
        trapdoory = hardwareMap.get(Servo.class, "trapdoory");
        outakeflipL = hardwareMap.get(Servo.class, "outakeflipL");
        outakeL = hardwareMap.get(DcMotor.class, "outakeL");
        color_sensor1 = hardwareMap.get(ColorSensor.class, "color_sensor1");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        // Put initialization blocks here.
        speed = 0.8;
        outakeR.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        pin.setDirection(Servo.Direction.FORWARD);
        pin.setPosition(0.7);
        trapdoory.setPosition(0.3);
        outakeflipL.setPosition(0.7);
        GateTime = new ElapsedTime();
        PurpleGate = false;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.right_trigger > 0.1) {
                    intake.setPower(1);
                    outakeL.setPower(0);
                    outakeR.setPower(0);
                } else {
                    intake.setPower(0);
                }
                if (gamepad1.left_trigger > 0.1) {
                    intake.setPower(-1);
                } else if (false) {
                    intake.setPower(0);

                } else {
                    Sticks(speed);
                }
                if (gamepad1.left_bumper) {
                    intake.setPower(1);
                } else if (false) {
                    intake.setPower(0);
                }
                if (gamepad1.aWasPressed()) {
                    outakeL.setPower(0);
                    outakeR.setPower(0);
                }
                if (gamepad1.xWasPressed()) {
                    outakeL.setPower(0.435);
                    outakeR.setPower(0.435);
                }
                if (gamepad1.yWasPressed()) {
                    outakeL.setPower(0.62);
                    outakeR.setPower(0.62);
                }
                if (gamepad1.bWasPressed()) {
                    outakeL.setPower(0.75);
                    outakeR.setPower(0.75);
                }
                if (gamepad1.bWasPressed()) {
                    outakeL.setPower(0);
                    outakeR.setPower(0);
                }
                if (gamepad1.dpad_down) {
                    outakeflipL.setPosition(0.4);
                } else if (gamepad1.dpad_down != true) {
                    outakeflipL.setPosition(0.715);
                }
                if (gamepad1.dpad_right) {
                    trapdoory.setPosition(0.6);
                } else if (gamepad1.dpad_right != true) {
                    trapdoory.setPosition(0.26);
                }
                myNormalizedColors = ((NormalizedColorSensor) color_sensor1).getNormalizedColors();
                Blue = myNormalizedColors.blue;
                Green = myNormalizedColors.green;
                Red = myNormalizedColors.red;
                if (Blue > Green) {
                    pin.setPosition(0.7);
                    Color2 = "Purple";
                    PurpleGate = true;
                    GateTime.reset();
                }
                if (Green > Blue && PurpleGate == false) {
                    pin.setPosition(0.5);
                    Color2 = "Green";
                }
                if (PurpleGate == true) {
                    pin.setPosition(0.7);
                }
                if (GateTime.seconds() >= 0.5 && PurpleGate == true) {
                    GateTime.reset();
                    PurpleGate = false;
                }
            }
        }
        // Put initialization blocks here.
    }

    /**
     * Describe this function...
     */
    private void Sticks(double speed) {
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        fl.setPower(speed * gamepad1.right_stick_x + (speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
        fr.setPower(-speed * gamepad1.right_stick_x + (-(speed * gamepad1.left_stick_x) - speed * gamepad1.left_stick_y));
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        bl.setPower(speed * gamepad1.right_stick_x + (-speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
        br.setPower(-speed * gamepad1.right_stick_x + (speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
    }

    /**
     * Describe this function...
     */
    private void do_something4() {
    }

    /**
     * Describe this function...
     */
    private void ResetEncoder() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void volor() {
    }

    /**
     * Describe this function...
     */
    private void do_something3() {
    }

    /**
     * Describe this function...
     */
    private void do_something() {
    }

    /**
     * Describe this function...
     */
    private void do_something2() {
    }
}