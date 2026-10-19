package org.firstinspires.ftc.teamcode.ArcPlanner.Hardware;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ArcPlanner.Core.ArcPose;

/**
 * Clase encargada de gestionar el odómetro goBILDA Pinpoint para el seguimiento
 * de la posición y orientación (pose) del robot en el campo.
 */
public class ArcPoint
{
    // Driver del sensor de odometría goBILDA Pinpoint
    private GoBildaPinpointDriver pinpoint;

    // Objeto que almacena la pose actual del robot (coordenadas X, Y y orientación heading)
    private ArcPose currentPose;

    /**
     * Constructor para inicializar el sensor Pinpoint, crear la pose inicial
     * y ejecutar la configuración de odometría.
     */
    public ArcPoint(GoBildaPinpointDriver pinpoint)
    {
        this.pinpoint = pinpoint;
        this.currentPose = new ArcPose();

        configureOdo();
    }

    /**
     * Configura los parámetros iniciales del sensor Pinpoint: tipo de pod de odometría,
     * offsets físicos de montaje, direcciones de los encoders y reinicio de posición/IMU.
     */
    private void configureOdo()
    {
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0, -180, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }

    /**
     * Actualiza los datos leyendo el sensor Pinpoint y mapeando la posición obtenida
     * a la estructura de pose interna del sistema en milímetros y radianes.
     */
    public void update()
    {
        pinpoint.update();
        Pose2D sensorPose = pinpoint.getPosition();

        currentPose.x = sensorPose.getX(DistanceUnit.MM);
        currentPose.y = sensorPose.getY(DistanceUnit.MM);
        currentPose.heading = sensorPose.getHeading(AngleUnit.RADIANS);
    }

    /**
     * Retorna la pose actual registrada del robot.
     */
    public ArcPose getPose()
    {
        return currentPose;
    }

    /**
     * Restablece la posición y la IMU del sensor Pinpoint a los valores iniciales (cero)
     * y reinicia la pose interna del robot.
     */
    public void reset()
    {
        pinpoint.resetPosAndIMU();
        currentPose = new ArcPose();
    }
}