/********************************************************
 *                                                      *
 *                                                      *
 *      user_sm.h			          	*
 *                                                      *
 *							*
 *		FI-UNAM					*
 *			                                *
 *                                                      *
 ********************************************************/
//#include "bug1.h"
// State Machine
int user_sm2(float intensity, float *light_values, float *observations,
			int size, float laser_value, int dest, int obs,
			movement *movements, int *next_state, float Mag_Advance,
			float max_twist, float rrobot, int *hab1, int *hab2, float *qx, float *qy,
			float robot_x, float robot_y, int *regreso)
{

	int state = *next_state;
	int result;
	int i, direccionluz, direccionobs;
	float giroA;
	float giroR, giro;
	float giroA2, giroR2;
	float val;
	float avance;
	    float qLx;
    float qLy;
    float qHx;
    float qHy;
    float distanciaLuz;
    int regreso2;
    int habobs;
    int habobs2;
	result = 0;
	*hab1 = *hab1 + 1;
	if (intensity > 29)
	{
		movements->twist = 0.0;
		movements->advance = 0.0;
		result = 1;
		return result;
		printf("\n **************** Reached light source ******************************\n");
	}
	else
	{
		printf("intensity %f\n", intensity);
		printf("quantized destination %d\n", dest);
		printf("quantized obs %d\n", obs);

		for (int i = 0; i < 8; i++)
		{
			// printf("light_values[%d] %f\n",i,light_values[i]);
			if (i == 0)
			{
				direccionluz = 0;
				val = light_values[i];
			}
			else if (light_values[i] < val)
			{
				// Guarda el valor menor, es el sensor más cercano a la luz
				direccionluz = i;
				val = light_values[i];
			}
		}
		printf("Min light %d\n", direccionluz);
		for (int i = 0; i < size; i++)
		{
			//  printf("laser observations[%d] %f\n",i,observations[i]);
			if (i == 0)
			{
				direccionobs = 0;
				val = observations[i];
			}
			else if (observations[i] < val)
			{
				// Busca menor valor, sensor más cercano a obstáculo
				direccionobs = i;
				val = observations[i];
			}
		}
		printf("Min obs %d\n", direccionobs);

		switch (direccionluz)
		{
		case 0:
			giroA = 4;
			break;

		case 1:
			giroA = 3;
			break;

		case 2:
			giroA = 2;
			break;

		case 3:
			giroA = 1;
			break;

		case 4:
			giroA = 0;
			break;

		case 5:
			giroA = -1;
			break;

		case 6:
			giroA = -2;
			break;

		case 7:
			giroA = -3;
			break;

		default:
			break;
		}
		float xobs = observations[direccionobs] * cos((direccionobs) * (3.1416 / 2) / size - 3.1416 / 4);
		float yobs = observations[direccionobs] * sin((direccionobs) * (3.1416 / 2) / size - 3.1416 / 4);
		float mag = sqrt(xobs * xobs + yobs * yobs);
		float frepx = (1 / mag - 1 / laser_value) * (1 / (mag * mag)) * (-xobs / mag);
		float frepy = (1 / mag - 1 / laser_value) * (1 / (mag * mag)) * (-yobs / mag);

		giroA = -giroA * (3.141592654 / 16); // Establece un giro de atracción a la luz, en términos de 45°

		printf("giroA %f, giroR %f, dirObs %d\n", giroA, giroR, direccionobs);
		printf("distancia a obstaculo %f", observations[direccionobs]);
		if (obs == 0)
		{ // Si no hay obstaculo solo gira hacia la luz
			giro = giroA;
		}
		else
		{ // Sino hace una ponderación
			// giro=(giroA)+giroR;
			if (*qx == 0)
			{
				*qx = robot_x;
				*qy = robot_y;
				*hab2 = *hab1;
			}
			float girox = cos(giroA) * (30 - intensity) + frepx * 0.0018;
			float giroy = sin(giroA) * (30 - intensity) + frepy * 0.0018;
			giro = atan2(giroy, girox);
		}
		printf("checkpoint qx %f,qy %f\n", *qx, *qy);
		printf("Giro= %f\n", giro);
		// if(giro==0||giro==giroR/2)
		//   giro=giroR;
		if (fabs(*hab2 - *hab1) > 410 && (fabs(*qx - robot_x) > 0.1) && (fabs(*qy - robot_y) > 0.1))
		{
			*qx = 0;
			*qy = 0;
			*regreso = 0;
			*hab2 = *hab1;
		}
		if (fabs(*hab2 - *hab1) >40 && (fabs(*qx - robot_x) < 0.06 && fabs(*qy - robot_y) < 0.06))
		{
			printf("voy de regreso qx %f,qy %f\n", *qx, *qy);
			*regreso = 1;
			// *movements = generate_output(BACKWARD, Mag_Advance, max_twist);
		}
		printf("regreso= %d\n", *regreso);
		printf("pasos= %d, %d", *hab1, *hab2);
		qLx=0;qLy=0;qHx=0;qHy=0;distanciaLuz=0;
		regreso2=0;habobs=0;habobs2=0;
		if (*regreso == 1)
		{
			if (intensity > 29)
			{

				movements->twist = 0.0;
				movements->advance = 0.0;
				result = 1;
				return result;
				printf("\n **************** Reached light source ******************************\n");
			}
			return -2;
			// int user_sm(float intensity, float *light_values, float *observations,
			// 			int size, float laser_value, int dest, int obs,
			// 			movement *movements, int *next_state, float Mag_Advance,
			// 			float max_twist, float rrobot, int *hab1, int *hab2, float *qx, float *qy,
			// 			float robot_x, float robot_y, int *regreso)

				// bug_1(intensity, light_values, observations, size,laser_value,
				// 	  dest, obs, movements, next_state, Mag_Advance, max_twist,robot_x,
				// 	  robot_y, &qLx, &qLy, &qHx, &qHy, &distanciaLuz, &regreso2, &habobs, &habobs2);

			//sm_avoidance_destination(intensity, dest, obs, movements, next_state, Mag_Advance, max_twist);
		}
		else
		{
			switch (*next_state)
			{
			case 0:
				// if(obs!=0 && (observations[direccionobs]-rrobot)<laser_value/2){
				// avance=Mag_Advance*(laser_value-observations[direccionobs])/laser_value;
				// }else{
				// avance=Mag_Advance/3;}
				avance = Mag_Advance / 2.5;
				movements->twist = giro;
				movements->advance = avance;
				break;

			case 1:
				// if(obs!=0 && (observations[direccionobs]-rrobot)<laser_value/2){
				// avance=Mag_Advance*(laser_value-observations[direccionobs])/laser_value;
				// }else{
				// avance=Mag_Advance/3;}
				avance = Mag_Advance / 2.5;
				//*movements=generate_output(FORWARD,Mag_Advance/2,max_twist);
				*movements = generate_output(FORWARD, avance, max_twist);
				*next_state = 0;
				break;
			default:
				break;
			}
		}

		printf("Next State: %d\n", *next_state);
	}
	return result;
}