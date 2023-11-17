
/********************************************************
 *                                                      *
 *                                                      *
 *      user_sm.h			          	*
 *                                                      *
 *							*
 *		FI-UNAM					*
 *		17-2-2019                               *
 *                                                      *
 ********************************************************/
#include <math.h>

// State Machine
int bug_1(
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
	int *habobs,
	int *habobs2)
{

	int state = *next_state;
	int i;
	int result = 0;
	printf("bug 1\n");
	//  printf("quantized destination %d\n",dest);
	*habobs = *habobs + 1;
	printf("quantized obs %d\n", obs);
	printf("abs1 %d\n", *habobs);
	printf("abs2 %d\n", *habobs2);
	printf("qHx %f\n", *qHx);
	printf("regreso %d\n", *regreso);

	//  for(int i = 0; i < 8; i++)
	//         printf("light_values[%d] %f\n",i,light_values[i]);
	//  for (int i = 0; i < size ; i++ )
	//          printf("laser observations[%d] %f\n",i,observations[i]);
	if (intensity > 30)
	{
		result = 1;
		return result;
		movements->twist = 0.0;
		movements->advance = 0.0;
		printf("\n **************** Reached light source ******************************\n");
		// stop();
	}
	if (*regreso == 1)
	{
		printf("ya voy de regreso\n");
	}
	switch (obs)
	{
	case 0:
	{
		if (fabs(*qLx - robot_x) < 0.05 && fabs(*qLy - robot_y) < 0.05 && *regreso == 1 && fabs(*habobs2 - *habobs) > 50)
		{
			printf("siguiente obstaculo\n");
			*qHx = 0;
			*qHy = 0;
			*regreso = 0;
			*movements = generate_output(LEFT, Mag_Advance, max_twist);
		}
		if (*qHx == 0)
		{
			printf("Seguir luz\n");
			int flg_result = light_follower(intensity, light_values, movements, Mag_Advance, max_twist);
			if (flg_result == 1)
				stop();
		}
		else
		{
			printf("rodeo de ostaculo\n");

			if (state == 0)
			{
				*movements = generate_output(RIGHT, Mag_Advance, max_twist / 5);
				*next_state = 2;
			}
			else
			{
				*movements = generate_output(FORWARD, Mag_Advance / 5, max_twist);
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
			printf("colicion en %f\n", *qHx);
			*habobs2 = *habobs;
		}
		else
		{
			printf("qhx %f robot_x %f\n", *qHx, robot_x);
			if ((fabs(*qHx - robot_x) < 0.08) && (fabs(*qHy - robot_y) < 0.08) && fabs(*habobs2 - *habobs) > 60)
			{
				printf("regrese a qH\n");
				*regreso = 1;
				//*movements=generate_output(RIGHT,Mag_Advance,max_twist*2);
			}
		}

		if (*distancialuz < intensity)
		{
			*qLx = robot_x;
			*qLy = robot_y;
			*distancialuz = intensity;
			printf("nuevo punto qL\n");
		}
		if (fabs(*qLx - robot_x) < 0.05 && fabs(*qLy - robot_y) < 0.05 && *regreso == 1 && fabs(*habobs2 - *habobs) > 50)
		{
			printf("siguiente obstaculo\n");
			*qHx = 0;
			*qHy = 0;
			*regreso = 0;
			*movements = generate_output(LEFT, Mag_Advance, max_twist);
		}
		else
		{
			if (observations[4] == laser_value)
				*movements = generate_output(FORWARD, Mag_Advance / 3.5, max_twist);
			else
				*movements = generate_output(LEFT, Mag_Advance, max_twist / 10);
		}
		// if(dest==3 && fabs(*habobs2 - *habobs) > 50){
		// 	printf("siguiente obstaculo\n");
		// 	*qHx = 0;
		// 	*qHy = 0;
		// 	*regreso = 0;
		// 	*movements = generate_output(LEFT, Mag_Advance, max_twist);
		// }
		break;
	}
	case 2:
		if (*qHx == 0)
		{
			*qHx = robot_x;
			*qHy = robot_y;
			printf("colicion en %f\n", *qHx);
			*habobs2 = *habobs;
		}
		*movements = generate_output(LEFT, Mag_Advance, max_twist);
		break;

	case 3:
	{
		if (*qHx == 0)
		{
			*qHx = robot_x;
			*qHy = robot_y;
			printf("colicion en %f\n", *qHx);
			*habobs2 = *habobs;
		}
		*movements = generate_output(LEFT, Mag_Advance, max_twist);
		break;
	}

	default:
		break;
	}

	return result;
}
