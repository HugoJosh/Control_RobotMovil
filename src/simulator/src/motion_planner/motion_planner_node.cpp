/***********************************************
 *                                              *
 *      motion_planner_node.cpp                 *
 *                                              *
 *      Jesus Savage                            *
 *      Diego Cordero                           *
 *                                              *
 *              Bio-Robotics Laboratory         *
 *              UNAM, 17-2-2020                 *
 *                                              *
 *                                              *
 ************************************************/

#include "ros/ros.h"
#include "../utilities/simulator_structures.h"
#include "../utilities/random.h"
#include "motion_planner_utilities.h"
#include "../state_machines/light_follower.h"
#include "../state_machines/sm_avoidance.h"
#include "../state_machines/sm_avoidance_destination.h"
#include "../state_machines/sm_destination.h"
#include "../state_machines/user_sm.h"
#include "../state_machines/user_sm2.h"
#include "../state_machines/dijkstra.h"
#include "../state_machines/dfs.h"
#include "../state_machines/campospotenciales.h"
#include "clips_ros/SimuladorRepresentation.h"
#include "../data/behaviors/oracle.h" //Está mal
#include "../action_planner/action_planner.h"
#include "../state_machines/bug1.h"
#include "../state_machines/bug2.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator_motion_planner_node");
    ros::NodeHandle n;
    ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub", 0, parametersCallback);
    ros::Subscriber sub = n.subscribe("/scan", 10, laserCallback);
    SimuladorRepresentation::setNodeHandle(&n);
    ros::Rate r(20);

    float qLx;
    float qLy;
    float qHx;
    float qHy;
    float distanciaLuz;
    float pend;
    float pend2;
    int regreso;
    int habobs;
    int habobs2;
    int habobs3;
    int habobs4;

    float lidar_readings[512];
    float light_readings[8];

    int i;
    int tmp;
    int sensor;
    int est_sig;
    int q_light;
    int q_inputs;
    int flagOnce;
    int flg_finish;
    int mini_sm = 1;
    int cta_steps;
    int flg_result;
    int flg_noise = 0;

    bool navegar = false;
    float x_nP;
    float y_np;

    float result;
    float final_x;
    float final_y;
    float intensity;
    float max_advance;
    float max_turn_angle;
    float noise_advance;
    float noise_angle;

    char path[100];
    char object_name[20];

    movement movements;
    step steps[200];
    step graph_steps[200];

    // it sets the environment's path
    strcpy(path, "./src/simulator/src/data/");

    while (ros::ok())
    {
        flagOnce = 1;
        cta_steps = 0;
        mini_sm = 1;

        while (params.run)
        {
            // it gets sensory data
            ros::spinOnce();

            if (!params.useRealRobot)
            {
                get_light_values(&intensity, light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h

                get_lidar_values(lidar_readings, params.robot_x,
                                 params.robot_y, params.robot_theta, params.useRealRobot); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            }
            else
            {
                get_light_values_RealRobot(&intensity, light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
                for (i = 0; i < 512; i++)
                    lidar_readings[i] = lasers[i];
            }

            // it quantizes the sensory data
            q_light = quantize_light(light_readings); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h

            if (params.noise)
                q_inputs = quantize_laser_noise(lidar_readings, params.laser_num_sensors, params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h
            else
                q_inputs = quantize_laser(lidar_readings, params.laser_num_sensors, params.laser_value); // function in ~/catkin_ws/src/simulator/src/motion_planner/motion_planner_utilities.h

            max_advance = params.robot_max_advance;
            max_turn_angle = params.robot_turn_angle;

            switch (params.behavior)
            {

            case 1:
                // This function sends light sensory data to a function that follows a light source and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/light_follower.h
                flg_result = light_follower(intensity, light_readings, &movements, max_advance, max_turn_angle);
                if (flg_result == 1)
                    stop();
                break;

            case 2:
                // This function sends light sensory data to an state machine that follows a light source and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_destination.h
                if (flagOnce)
                {
                    est_sig = 1;
                    flagOnce = 0;
                }
                flg_result = sm_destination(intensity, q_light, &movements, &est_sig, params.robot_max_advance, params.robot_turn_angle);
                if (flg_result == 1)
                    stop();

                break;

            case 3:
                // This function sends quantized sensory data to an state machine that avoids obstacles and it issues
                // the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_avoidance.h
                if (flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                sm_avoid_obstacles(q_inputs, &movements, &est_sig, params.robot_max_advance, params.robot_turn_angle);
                break;

            case 4:
                // This function sends quantized sensory data to an state machine that follows a light source and avoids obstacles
                // and it issues the actions that the robot needs to perfom.
                // It is located in ~/catkin_ws/src/simulator/src/state_machines/sm_avoidance_destination.h
                if (flagOnce)
                {
                    est_sig = 0;
                    flagOnce = 0;
                }
                flg_result = sm_avoidance_destination(intensity, q_light, q_inputs, &movements, &est_sig,
                                                      params.robot_max_advance, params.robot_turn_angle);

                if (flg_result == 1)
                    stop();
                break;

            case 5:
                // This function sends the intensity and the quantized sensory data to a Clips node and it receives the actions that
                // the robot needs to perfom to avoid obstacles and reach a light source according to first order logic
                // It is located in ~/catkin_ws/src/simulator/src/behaviors/oracle.h
                result = oracle_clips(intensity, q_light, q_inputs, &movements, max_advance, max_turn_angle);
                if (result == 1.0)
                    stop();
                break;

            case 6:
                // it finds a path from the origen to a destination using depth first search
                if (flagOnce)
                {
                    for (i = 0; i < 200; i++)
                        steps[i].node = -1;

                    // it finds a path from the origen to a destination using first search
                    dfs(params.robot_x, params.robot_y, params.light_x, params.light_y, params.world_name, steps);
                    print_algorithm_graph(steps);
                    i = 0;
                    final_x = params.light_x;
                    final_y = params.light_y;
                    set_light_position(steps[i].x, steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n", i, steps[i].x, steps[i].y);
                    flagOnce = 0;
                    flg_finish = 0;
                    est_sig = 0;
                    movements.twist = 0.0;
                    movements.advance = 0.0;
                }
                else
                {
                    // flg_result=sm_avoidance_destination(intensity,q_light,q_inputs,&movements,&est_sig,                                                        //params.robot_max_advance ,params.robot_turn_angle);
                    flg_result = oracle_clips(intensity, q_light, q_inputs, &movements, max_advance, max_turn_angle);

                    if (flg_result == 1)
                    {
                        if (flg_finish == 1)
                            stop();
                        else
                        {
                            if (steps[i].node != -1)
                            {
                                set_light_position(steps[i].x, steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n", i, steps[i].x, steps[i].y);
                                printf("Node %d\n", steps[i].node);
                                i++;
                                // printf("type a number \n");
                                // scanf("%d",&tmp);
                            }
                            else
                            {
                                set_light_position(final_x, final_y);
                                printf("New Light %d: x = %f  y = %f \n", i, final_x, final_y);
                                flg_finish = 1;
                            }
                        }
                    }
                }

                break;

            case 7:
                if (flagOnce)
                {
                    for (i = 0; i < 200; i++)
                        steps[i].node = -1;
                    // it finds a path from the origen to a destination using the Dijkstra algorithm
                    dijkstra(params.robot_x, params.robot_y, params.light_x, params.light_y, params.world_name, steps);
                    print_algorithm_graph(steps);
                    i = 0;
                    final_x = params.light_x;
                    final_y = params.light_y;
                    set_light_position(steps[i].x, steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n", i, steps[i].x, steps[i].y);
                    flagOnce = 0;
                    flg_finish = 0;
                    est_sig = 0;
                    movements.twist = 0.0;
                    movements.advance = 0.0;
                }
                else
                {
                    flg_result = sm_avoidance_destination(intensity, q_light, q_inputs, &movements, &est_sig,
                                                          params.robot_max_advance, params.robot_turn_angle);

                    if (flg_result == 1)
                    {
                        if (flg_finish == 1)
                            stop();
                        else
                        {
                            if (steps[i].node != -1)
                            {
                                set_light_position(steps[i].x, steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n", i, steps[i].x, steps[i].y);
                                printf("Node %d\n", steps[i].node);
                                i++;
                                // printf("type a number \n");
                                // scanf("%d",&tmp);
                            }
                            else
                            {
                                set_light_position(final_x, final_y);
                                printf("New Light %d: x = %f  y = %f \n", i, final_x, final_y);
                                flg_finish = 1;
                            }
                        }
                    }
                }
                break;

            case 8:
                // Inicialización de parámetros
                if (flagOnce)
                {
                    for (i = 0; i < 200; i++)
                        steps[i].node = -1;
                    user(params.robot_x, params.robot_y, params.light_x, params.light_y, params.world_name, steps);
                    printf("Los pasos a seguir para encontrar el grafo: \n");
                    print_algorithm_graph(steps);
                    i = 0;
                    final_x = params.light_x;
                    final_y = params.light_y;
                    set_light_position(steps[i].x, steps[i].y);
                    printf("New Light %d: x = %f  y = %f \n", i, steps[i].x, steps[i].y);
                    flagOnce = 0;
                    flg_finish = 0;
                    est_sig = 0;
                    movements.twist = 0.0;
                    movements.advance = 0.0;
                }
                else
                {
                    printf("Turn_angle: %f robot theta: %f", params.robot_turn_angle, params.robot_theta);
                    flg_result = camposPotenciales(intensity, lidar_readings, params.laser_num_sensors, params.laser_value,
                                                   &movements, params.light_x, params.light_y, params.robot_x, params.robot_y, params.robot_theta);
                    if (flg_result == 1)
                    {
                        // Continua moviendote en el siguinte nodo, 1 indica que llegó.
                        if (flg_finish == 1)
                            stop();
                        else
                        {
                            if (steps[i].node != -1)
                            {
                                set_light_position(steps[i].x, steps[i].y);
                                printf("New Light %d: x = %f  y = %f \n", i, steps[i].x, steps[i].y);
                                printf("Node %d\n", steps[i].node);
                                i++;
                            }
                            else
                            {
                                set_light_position(final_x, final_y);
                                printf("New Light %d: x = %f  y = %f \n", i, final_x, final_y);
                                flg_finish = 1;
                            }
                        }
                    }
                }

                break;

            case 9:

                flg_result = light_follower(intensity, light_readings, &movements, max_advance, max_turn_angle);
                if (flg_result == 1)
                    set_light_position(.5, .25);

                break;

            case 10:

                if (navegar)
                {
                    if (flagOnce)
                    {
                        for (i = 0; i < 200; i++)
                            steps[i].node = -1;
                        dijkstra(params.robot_x, params.robot_y, x_nP, y_np, params.world_name, steps);
                        // user(params.robot_x ,params.robot_y, x_nP, y_np ,params.world_name,steps);
                        print_algorithm_graph(steps);
                        i = 0;
                        final_x = params.light_x;
                        final_y = params.light_y;
                        set_light_position(steps[i].x, steps[i].y);
                        printf("New Light %d: x = %f  y = %f \n", i, steps[i].x, steps[i].y);
                        flagOnce = 0;
                        flg_finish = 0;
                        est_sig = 0;
                        movements.twist = 0.0;
                        movements.advance = 0.0;
                        qLx = 0;
                        qLy = 0;
                        qHx = 0;
                        qHy = 0;
                        distanciaLuz = 0;
                        regreso = 0;
                        habobs = 0;
                        habobs2 = 0;
                        // pend = (params.light_y - params.robot_y) / (params.light_x - params.robot_x);
                        // pend2 = atan2((params.light_y - params.robot_y), (params.light_x - params.robot_x));
                    }
                    else
                    {
                        // printf("Turn_angle: %f robot theta: %f", params.robot_turn_angle, params.robot_theta);
                        // flg_result = sm_avoidance_destination(intensity, q_light, q_inputs, &movements, &est_sig,
                        //                                       params.robot_max_advance, params.robot_turn_angle);
                        flg_result = user_sm2(intensity, light_readings, lidar_readings, params.laser_num_sensors, params.laser_value,
                                             q_light, q_inputs, &movements, &est_sig, params.robot_max_advance, params.robot_turn_angle, params.robot_radio, &habobs, &habobs2, &qHx, &qHy, params.robot_x, params.robot_y, &regreso);
                        if (flg_result == -2)
                        {
                          flg_result = bug_1(intensity, light_readings, lidar_readings, params.laser_num_sensors, params.laser_value,
                                           q_light, q_inputs, &movements, &est_sig, params.robot_max_advance, params.robot_turn_angle,
                                           params.robot_x, params.robot_y, &qLx, &qLy, &qHx, &qHy, &distanciaLuz, &regreso, &habobs, &habobs2);    
                        }
                        // flg_result = bug_1(intensity, light_readings, lidar_readings, params.laser_num_sensors, params.laser_value,
                        //                    q_light, q_inputs, &movements, &est_sig, params.robot_max_advance, params.robot_turn_angle,
                        //                    params.robot_x, params.robot_y, &qLx, &qLy, &qHx, &qHy, &distanciaLuz, &regreso, &habobs, &habobs2);

                        // flg_result=camposPotenciales(intensity,lidar_readings, params.laser_num_sensors, params.laser_value,
                        //             &movements, params.light_x ,params.light_y, params.robot_x, params.robot_y,  params.robot_theta);

                        // pend = (params.light_y - params.robot_y) / (params.light_x - params.robot_x);
                        // pend2 = atan2((params.light_y - params.robot_y), (params.light_x - params.robot_x));
                        // flg_result = bug_2(intensity, light_readings, lidar_readings, params.laser_num_sensors, params.laser_value,
                        //                    q_light, q_inputs, &movements, &est_sig, params.robot_max_advance, params.robot_turn_angle,
                        //                    params.robot_x, params.robot_y, &qLx, &qLy, &qHx, &qHy, &distanciaLuz, &regreso, params.light_x, params.light_y, pend, params.robot_theta, pend2);
                        if (flg_result == 1)
                        {
                            // Continua moviendote en el siguinte nodo, 1 indica que llegó.
                            if (flg_finish == 1)
                            {
                                stop();
                                navegar = false;
                            }
                            else
                            {
                                qLx = 0;
                                qLy = 0;
                                qHx = 0;
                                qHy = 0;
                                distanciaLuz = 0;
                                regreso = 0;
                                habobs = 0;
                                habobs2 = 0;
                                pend = (params.light_y - params.robot_y) / (params.light_x - params.robot_x);
                                pend2 = atan2((params.light_y - params.robot_y), (params.light_x - params.robot_x));
                                if (steps[i].node != -1)
                                {
                                    set_light_position(steps[i].x, steps[i].y);
                                    printf("New Light %d: x = %f  y = %f \n", i, steps[i].x, steps[i].y);
                                    printf("Node %d\n", steps[i].node);
                                    i++;
                                }
                                else
                                { // Es el último nodo, ya no hay más
                                    set_light_position(final_x, final_y);
                                    printf("New Light %d: x = %f  y = %f \n", i, final_x, final_y);
                                    flg_finish = 1;
                                    navegar = false;
                                    flagOnce = 1;
                                }
                            }
                        }
                    }
                }
                else
                {
                    navegar = action_planner(params.robot_x, params.robot_y, params.robot_theta, &movements, &x_nP, &y_np);
                    printf("\n\n\nNueva acción de CLIPS: %d, posición dada: %f,  %f \n\n", navegar, x_nP, y_np);
                    final_x = params.light_x;
                    final_y = params.light_y;
                }

                break;

            default:
                printf(" ******* SELECTION NO DEFINED *******\n");
                movements.twist = 3.1416 / 4;
                movements.advance = .03;
                break;
            }

            ros::spinOnce();
            printf("\n\n             MOTION PLANNER \n________________________________\n");
            printf("Light: x = %f  y = %f \n", params.light_x, params.light_y);
            printf("Robot: x = %f  y = %f \n", params.robot_x, params.robot_y);
            printf("Step: %d \n", cta_steps++);
            printf("Movement: twist: %f advance: %f \n", movements.twist, movements.advance);

            flg_noise = params.noise;

            move_robot(movements.twist, movements.advance, lidar_readings);
            ros::spinOnce();
            new_simulation = 0;
            r.sleep();
        }
        ros::spinOnce();
        r.sleep();
    }
}
