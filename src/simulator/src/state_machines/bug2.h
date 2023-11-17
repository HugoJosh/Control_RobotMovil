#include <math.h>
// State Machine
int bug_2(
	float intensity,
	float *light_values,
	float *observations,
	int size,
	float laser_value,
	int dest,
	int obs,
	movement *movements,
	int *next_state,
	float Mag_Advance,
	float max_twist,
	float robot_x,
	float robot_y,
	float *qLx,
	float *qLy,
	float *qHx,
	float *qHy,
	float *distancialuz,
	int *regreso,
	float x0,
	float y0,
	float m,
	float theta,
	float thetao)
{
	float angulo;
	int state = *next_state;
	int i;
	int obs2 = obs;
	float y;
	printf("intensity %f\n", intensity);
	printf("theta %f\n", theta);
	printf("theta0 %f\n", thetao);
	angulo = thetao - theta;
	// estimacion de posicion
	if (fabs(m) < 30)
	{
		y = m * (robot_x - x0) + y0;
		printf("y %f\n", y);
		printf("robot_y %f\n", robot_y);
		printf("m %f\n", m);
		printf("m %f\n", m);
		printf("robot_x %f\n", robot_x);
	}
	else
	{
		y = robot_y;
		printf("m %f\n", m);
		printf("robot_x %f\n", robot_x);
		printf("y %f\n", y);
		printf("robot_y %f\n", robot_y);
	}

	float x = (robot_y - y0) / m + x0;
	if (fabs(m) < 0.05)
	{
		x = robot_x;
	}
	printf("x %f\n", x);

	//  printf("quantized destination %d\n",dest);
	//  printf("quantized obs %d\n",obs);

	//  for(int i = 0; i < 8; i++)
	//         printf("light_values[%d] %f\n",i,light_values[i]);
	//  for (int i = 0; i < size ; i++ )
	//          printf("laser observations[%d] %f\n",i,observations[i]);
	// if(((fabs(y-robot_y)<0.08 ||fabs(x-robot_x)<0.08)&&fabs(m)<30&&fabs(m)>0.05&&fabs(robot_x-*qHx)>0.05&&fabs(robot_y-*qHy)>0.04)||(fabs(x-robot_x)<0.04&&fabs(m)>30&&fabs(robot_y-*qHy)>0.06)||(fabs(y-robot_y)<0.04&&fabs(m)<0.05&&fabs(robot_x-*qHx)>0.06))
	// {
	// 	obs2=0;
	// }
	if (*regreso == 1 && obs != 3)
	{
		obs2 = 0;
	}
	if ((((fabs(y - robot_y) < 0.05 || fabs(x - robot_x) < 0.05) && fabs(m) < 30 && fabs(m) > 0.05 && fabs(robot_x - *qHx) > 0.05 && fabs(robot_y - *qHy) > 0.04) || (fabs(x - robot_x) < 0.02 && fabs(m) > 30 && fabs(robot_y - *qHy) > 0.06) || (fabs(y - robot_y) < 0.02 && fabs(m) < 0.05 && fabs(robot_x - *qHx) > 0.06)) && *qHx != 0)
	{
		obs2 = 0;
		printf("siguiente obstaculo\n");
		*qHx = 0;
		*qHy = 0;
		*regreso = 0;
		if (state == 0)
		{
			*next_state = 2;
		}
		else
		{
			*next_state = 0;
		}
		//*movements=generate_output(LEFT,Mag_Advance,max_twist);
	}
	printf("obs2 %d\n", obs2);
	switch (obs2)
	{
	case 0:
	{
		if (*qHx == 0 || ((fabs(y - robot_y) < 0.05 || fabs(x - robot_x) < 0.05) && fabs(m) < 30 && fabs(m) > 0.05))
		{
			printf("Seguir luz\n");
			//   int flg_result = light_follower
			//   (intensity,light_values,movements,Mag_Advance,max_twist);
			//             if(flg_result == 1) stop();

			angulo = thetao - theta;
			printf("angulo %f\n",angulo);
			if (state == 0 && fabs(angulo) > 0.03)
			{
				angulo = thetao - theta;
				if (angulo > 0)
				{
					*movements = generate_output(LEFT, Mag_Advance, angulo);
				}
				else
				{
					*movements = generate_output(RIGHT, Mag_Advance, -angulo);
				}
				printf("AQUI 1\n");
				*next_state = 2;
			}
			else
			{
				printf("AQUI 2\n");
				if (observations[8] == laser_value && observations[11] == laser_value && observations[14] == laser_value && observations[5] == laser_value)
				{
					*movements = generate_output(FORWARD, Mag_Advance / 3, max_twist);
					*next_state = 0;
				}
			}
		}
		else
		{
			printf("rodeo de ostaculo\n");

			if (state == 0)
			{
				if (observations[5] == laser_value)
				{
					*movements = generate_output(RIGHT, Mag_Advance, max_twist / 6);
				}
				*next_state = 2;
				printf("AQUI 3\n");
			}
			else
			{
				*movements = generate_output(FORWARD, Mag_Advance / 7, max_twist);
				printf("AQUI 4\n");
				*next_state = 0;
			}
		}
	}
	break;
	case 1:
	{
		if (*qHx == 0)
		{
			*qHx = robot_x;
			*qHy = robot_y;
			*distancialuz = intensity;
			printf("colicion en %f\n", *qHx);
		}
		if ((((fabs(y - robot_y) < 0.05 || fabs(x - robot_x) < 0.05) && fabs(m) < 30 && fabs(m) > 0.05 && fabs(robot_x - *qHx) > 0.05 && fabs(robot_y - *qHy) > 0.04) || (fabs(x - robot_x) < 0.05 && fabs(m) > 30 && fabs(robot_y - *qHy) > 0.06) || (fabs(y - robot_y) < 0.05 && fabs(m) < 0.05 && fabs(robot_x - *qHx) > 0.06)))
		{
			*qHx = 0;
			*qHy = 0;
			*regreso = 1;
			//*next_state = 0;
		}
		else
		{
			if (observations[2] == laser_value && observations[4] == laser_value && observations[8] == laser_value && observations[11] == laser_value && observations[16] == laser_value && observations[18] == laser_value)
			{
				*movements = generate_output(FORWARD, Mag_Advance / 5, max_twist);
				printf("AQUI 5\n");
			}
			else
			{
				printf("AQUI 6\n");
				*movements = generate_output(LEFT, Mag_Advance, max_twist / 10);
			}
		}

		break;
	}
	case 2:
	{
		*movements = generate_output(LEFT, Mag_Advance, max_twist);
		break;
	}
	case 3:
	{
		if (*qHx == 0)
		{
			*qHx = robot_x;
			*qHy = robot_y;
			*distancialuz = intensity;
			printf("colicion en %f\n", *qHx);
		}
		*regreso = 0;
		*movements = generate_output(LEFT, Mag_Advance, max_twist);
		break;
	}

	default:
		break;
	}

	if (intensity > 29)
	{

		movements->twist = 0.0;
		movements->advance = 0.0;
		printf("\n **************** Reached light source ******************************\n");
		return 1;
	}
	return -1;
}
