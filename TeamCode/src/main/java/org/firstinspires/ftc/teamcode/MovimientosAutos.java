package org.firstinspires.ftc.teamcode;

//Importacion de librerias

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//Propiedades de visualizacion Driver Station.
@Autonomous(name="Auto Moves", group="Autos",preselectTeleOp="TitaniumRamsRegional")

//Clase principal del programa (IMU utilizado modelo:
public class MovimientosAutos extends LinearOpMode {

    // Declarar los objetos de motores
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private BHI260IMU imu;
    private Orientation angulos;
    private DistanceSensor distanceSensor;
    private double distancia;

    // Definir el objetivo en pulgadas
    // Definir la circunferencia del rueda en pulgadas
    private static final double WHEEL_CIRCUMFERENCE = 3.54331* Math.PI; // Diametro de las ruedas 3.54331 pulgadas

    // Define el encoder ticks por revolution (TPR) del motor
    private static final int TICKS_PER_REV =  420;//yMark NeveRest 40

    @Override
    public void runOpMode() {
        // inicializa las variables de hardware, los strings son los nombres de parametros en la driver station.
        // La funcion get obtiene el mapeo de la configuracion de hardware en el driver station.
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // Se establece la direccion de los motores.Debido a que los motores estan invertidos uno respeco al otro,
        // se invierte el motor derecho.

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Se establece el modo de freno al estar en neutral.
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Inicialización del IMU BHI260AP

        // Los parámetros son limitados en BHI260AP,
        // generalmente no se necesitan configuraciones adicionales.

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distancia");
        distancia = distanceSensor.getDistance(DistanceUnit.CM);

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters parametrosIMU = new BHI260IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT) );

                       
        imu.initialize(parametrosIMU);


        // Esperar a que el IMU esté listo (puede no ser necesario para BHI260AP)
        sleep(100); // Pausa corta para asegurar la inicialización

        telemetry.addData("IMU", "Inicializado");
        telemetry.addData("Distancia Init", distancia);
        telemetry.update();

        // La funcion waitForStart() espera a que el boton de play sea presionado.

        waitForStart();

        //===================SECUENCIA DE COMANDOS AUTONOMOS====================================

        Adelante(15,0.5,500);
        Atras(15,0.5,500);
        girarIzquierda(90,0.6,500); // Girar 90 grados a la izquierda
        girarDerecha(90,0.6,500);  // Girar 45 grados a la derecha















        //================================FUNCIONES DE MOVIMIENTOS=================================

    }  private void Adelante(double TARGET_DISTANCE ,double Power, long SLEEPTIME) {
        // Reset encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate target encoder ticks
        int targetTicks = (int) ((TARGET_DISTANCE / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV);

        // Set target position and run to position
        leftDrive.setTargetPosition(targetTicks);
        rightDrive.setTargetPosition(targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motors
        leftDrive.setPower(Power);
        rightDrive.setPower(Power);

        // Wait for motors to reach target position
        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            // Display telemetry data (optional)
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Left Position", leftDrive.getCurrentPosition());
            telemetry.addData("Right Position", rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        sleep(SLEEPTIME);
    }


    private void Atras(double TARGET_DISTANCE ,double Power, long SLEEPTIME) {
        TARGET_DISTANCE = -TARGET_DISTANCE;
        // Reset encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate target encoder ticks
        int targetTicks = (int) ((TARGET_DISTANCE / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV);

        // Set target position and run to position
        leftDrive.setTargetPosition(targetTicks);
        rightDrive.setTargetPosition(targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motors
        leftDrive.setPower(Power);
        rightDrive.setPower(Power);

        // Wait for motors to reach target position
        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            // Display telemetry data (optional)
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Left Position", leftDrive.getCurrentPosition());
            telemetry.addData("Right Position", rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        sleep(SLEEPTIME);
    }


    // Función para girar a la izquierda (grados positivos)
    private void girarIzquierda(double grados, double Power, long SLEEPTIME) { girar(grados,Power,SLEEPTIME);}

    // Función para girar a la derecha (grados negativos)
    private void girarDerecha(double grados, double Power, long SLEEPTIME) { girar(-grados,Power,SLEEPTIME); }

    // Función principal para realizar el giro
    private void girar(double grados, double Power, long SLEEPTIME) {

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //imu.resetYaw();
        angulos = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double anguloInicial = angulos.firstAngle;
        double anguloObjetivo = anguloInicial + grados;

        // Ajustar el ángulo objetivo para que esté entre -180 y 180 grados
        anguloObjetivo = anguloObjetivo % 360;
        if (anguloObjetivo > 180) {
            anguloObjetivo -= 360;
        } else if (anguloObjetivo < -180) {
            anguloObjetivo += 360;
        }

        while (opModeIsActive() && Math.abs(angulos.firstAngle - anguloObjetivo) > 1) {
            angulos = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double potencia = (grados > 0 ? Power : -Power); // Ajustar la potencia según la dirección

            leftDrive.setPower(-potencia);
            rightDrive.setPower(potencia);

            telemetry.addLine("==Angulo del IMU==");
            telemetry.addData("Ángulo actual", angulos.firstAngle);
            telemetry.addData("Ángulo objetivo", anguloObjetivo);
            telemetry.addLine("**Potencias del Drive**");
            telemetry.addData("LeftDrive", leftDrive.getPower());
            telemetry.addData("RightDrive", rightDrive.getPower());
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        sleep(SLEEPTIME);
    }
}




