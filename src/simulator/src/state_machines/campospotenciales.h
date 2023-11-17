#include <vector>
#include <math.h>
//const float PI = 3.1415926535;



float calculateAngle(std::vector<float> fuerzas, float robot_angle);
void normVector(float *observations, int size, float laser_value);
std::vector <float> calculateFuerzaRep(float *observations, int size, float laser_value, float robot_angle);
int camposPotenciales(float intensity, float *observations, int size,
  float laser_value, movement *movements, int dest_x, int dest_y, int robot_x, int robot_y, float robot_angle);


 int max_observation; //Index
 float max_obs; //Value
 float ep = 4.5; //Constante de fuerza de atracción
 float delta = 0.011;
 float eta = 9.5;//Constante de repulsión


int camposPotenciales(float intensity, float *observations, int size, float laser_value, movement *movements,
   float dest_x, float dest_y, float robot_x, float robot_y, float robot_angle){

 int i;
 float incX,incY, mag;
 std::vector<float> fAtr;
 std::vector<float> fRep;
 std::vector<float> fuerzas;

 if (intensity > 38.0) {
   movements->twist = 0;
   movements->advance = 0; //delta
   return 1;
 }

  fAtr = {dest_x - robot_x, dest_y - robot_y};
  mag = sqrt((fAtr[0]*fAtr[0])+(fAtr[1]*fAtr[1]));
  fAtr = {ep*fAtr[0]/mag, ep*fAtr[1]/mag};//Normalización vector
  fRep = calculateFuerzaRep(observations, size, laser_value, robot_angle);

  incY = 1.0;
  incX = 1.0;
  if (fabs(fRep[0]) > fabs(fRep[1]) && fRep[0] != 0) { //Mayor fuerza de repulsión en eje x
    incY = 3.0;
  }else if(fRep[0] != 0){
    incX = 3.0;
  }
  fuerzas = {incX*fAtr[0]+fRep[0], incY*fAtr[1]+fRep[1]};
  printf(" --> Vector fuerza atracción: %f %f\n", fAtr[0], fAtr[1]);
  printf(" --> Vector fuerza: %f %f\n", fuerzas[0], fuerzas[1]);
  printf("Punto de destino: %f %f\n", dest_x, dest_y);

  movements->twist = calculateAngle(fuerzas, robot_angle);
  movements->advance = delta; //delta
  return 0;
}






/*
Calcula el ańgulo de giro del vector de fuerzas reultante en el eje de referencia
del robot.
*/
float calculateAngle(std::vector<float> fuerzas, float robot_angle){
  float giro = atan2(fuerzas[1],fuerzas[0]);
  giro = giro - robot_angle;
  giro = giro > 2*PI ?  giro - 2*PI : giro;
  giro = giro < -2*PI ?  giro + 2*PI : giro;

  giro = giro > PI ? 2*PI - giro : giro;
  giro = giro < -PI ? 2*PI + giro : giro;
	printf("\n>>Cambio de ángulo %f \n\n",giro); //Padding de 90 para que no viaje perpendicular
	return (giro);
}



/*Calcula el vector de fuerza repulsiva utiliando una aproximación del vector
normal a el obstaculo*/

std::vector <float> calculateFuerzaRep(float *observations, int size, float laser_value, float robot_angle){
	std::vector <float> fuerzaRep;
  normVector(observations, size, laser_value);
	if (max_obs == laser_value){
		printf(" --> No hay fuerza de repulsión\n");
    return {0,0};
	}else{
		float scale = PI /size;
		float angle = (0 + scale*max_observation - PI/2); //Dirección del obstaculo eje x en frente robot
    float angle_real = robot_angle + angle; //ángulo del vector respecto al eje de referencia del mapa
		float mag = eta*(1 - (max_obs/laser_value));
    mag = eta*1;
    angle_real = angle_real > 2*PI ?  angle_real - 2*PI : angle_real;

		//Se normaliza a 1 y se aplica el coeficinete está invertido , cuando hay objeto decrementa
		fuerzaRep = {-mag*(float)cos(angle_real), -mag*(float)sin(angle_real)}; //Se va en sentido contario al objeto encontrado

    //Se añade un pad para que lo revote en digonal
		//fuerzaRep[0] = fuerzaRep[0] == 0? eta*2 : fuerzaRep[0];
		//fuerzaRep[1] = fuerzaRep[1] == 0? eta*2 : fuerzaRep[1];

		//fuerzaRep[0] = fabs(fuerzaRep[0]) < 2? fuerzaRep[0]*1.5 : fuerzaRep[0];
		//fuerzaRep[1] = fabs(fuerzaRep[1]) < 2? fuerzaRep[1]*1.5 : fuerzaRep[1];
		printf(" --> Vector de repulsión %f %f\n", fuerzaRep[0], fuerzaRep[1]);
    printf(" -----> Ángulo %f \n", angle_real);
    printf(" _____ Respecto al eje angle %f en sensor %d\n",angle, max_observation);
	}
	return fuerzaRep;
}



/*
Calcula el máximo y el mínimo para cada uno de los sensores de proximidad
Aproxima el vector normal a el obstaculo usando el valor máximo.
*/
void normVector(float *observations, int size, float laser_value){
	max_obs = laser_value;
	for (int i = 0; i < size ; i++ ){
		if(observations[i] < max_obs){ //cuando se acerca a un obstaculo se hae pequeño
			max_observation = i;
			max_obs = observations[i];
		}
	}
	printf("max observation[%d] %f\n", max_observation, max_obs);
}
