package org.firstinspires.ftc.teamcode.ArcPlanner.Path;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.ArcPlanner.Core.ArcPose;

/**
 * Clase encargada de administrar una secuencia de puntos de ruta (waypoints)
 * y de gestionar el progreso de navegación del robot a lo largo de un trayecto.
 */
public class ArcPath
{
    // Lista que almacena los waypoints del trayecto
    private List<ArcPose> wayPoints;

    // Índice del waypoint objetivo actual dentro de la lista
    private int currentWaypointIndex = 0;

    // Distancia de umbral (en milímetros) para considerar que se ha llegado a un waypoint y avanzar al siguiente
    private double arrivalThreshold = 20.0;

    /**
     * Constructor para inicializar una nueva ruta vacía.
     */
    public ArcPath()
    {
        this.wayPoints = new ArrayList<>();
    }

    /**
     * Agrega un nuevo waypoint (pose objetivo) al final de la ruta.
     */
    public void addWaypoint(ArcPose pose)
    {
        wayPoints.add(pose);
    }

    /**
     * Retorna el waypoint objetivo actual; si la ruta ha finalizado, retorna nulo.
     */
    public ArcPose getCurrentTarget()
    {
        if (isFinished()) return null;

        return wayPoints.get(currentWaypointIndex);
    }

    /**
     * Actualiza el progreso de la ruta evaluando la distancia entre el robot y el waypoint actual;
     * si se encuentra dentro del umbral de llegada, avanza al siguiente waypoint.
     */
    public void updateProgress(ArcPose currentRobotPose)
    {
        if (isFinished()) return;

        double distance = currentRobotPose.distanceTo(wayPoints.get(currentWaypointIndex));

        if (distance < arrivalThreshold)
        {
            currentWaypointIndex++;
        }
    }

    /**
     * Verifica si se han recorrido todos los waypoints de la ruta.
     */
    public boolean isFinished()
    {
        return currentWaypointIndex >= wayPoints.size();
    }

    /**
     * Reinicia el progreso de la ruta colocando el índice del waypoint actual de vuelta al inicio (0).
     */
    public void resetPath()
    {
        currentWaypointIndex = 0;
    }
}