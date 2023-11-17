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
#include <vector>
#include <math.h>
#include <queue>
#include <list>
#include "dijkstra.h"

using namespace std;

int user(float rx, float ry, float lx, float ly, char *world_name, step *steps);
void BFS_algorithm(int D, int L);
void A_star_algorithm(int D, int L);
bool setSuccessors(nodo *padre, nodo *node_goal, list<nodo *> &open);
nodo * getBetterNode(list<nodo *> &candidates);

const float PI = 3.1415926535;
const int th_min = 0.04;
extern nodo nodes[MUM_NODES];
extern int num_nodes;


// Crea la secuencia de nodos a seguir
int user(float rx, float ry, float lx, float ly, char *world_name, step *steps){
	char archivo[150];
	int i;
	int start = 0;
	int goal = 0;
	int padre;
	std::string paths = ros::package::getPath("simulator");
	strcpy(archivo,paths.c_str());
	strcat(archivo,"/src/data/");
	strcat(archivo,world_name);
	strcat(archivo,"/");
	strcat(archivo,world_name);
	strcat(archivo,".top");

	for(i = 0; i < MUM_NODES; i++)
	{
		 nodes[i].flag='N';
		 nodes[i].num_conections = 0;
		 nodes[i].parent = -1;
		 nodes[i].acumulado = 0;
	}

	num_nodes = read_nodes(archivo); // Se lee el arcivo .top

	//Establece cual es el nodo más cerano al objetivo y al robot
	for(i = 1; i < num_nodes; i++){
		 if( sqrt(pow( nodes[i].x - rx ,2) + pow( nodes[i].y - ry ,2)) < sqrt( pow( nodes[start].x - rx ,2) + pow( nodes[start].y - ry ,2)) )
			 start = i;

		 if( sqrt(pow( nodes[i].x - lx ,2) + pow( nodes[i].y - ly ,2)) < sqrt(pow( nodes[goal].x - lx ,2) + pow( nodes[goal].y - ly ,2) ) )
			 goal = i;
	}
	printf("Nodo inicial %d y nodo final %d\n", start, goal);
	A_star_algorithm(goal ,start);

	padre = start;
	printf("\n\nindice: %d, contenido:\n", padre);
	printNode(padre);

	i = 0;

	while( padre != -1)
	{
		steps[i].node = nodes[padre].num_node;
		steps[i].x = nodes[padre].x;
		steps[i].y = nodes[padre].y;
		i++;
		padre = nodes[padre].parent;
		printf("\nNodo: %d \n", nodes[padre].num_node);
		printf("Siguiente: %d\n", nodes[padre].parent);
	}
 	return 0;
}

void BFS_algorithm(int D, int L){
	/*
		 D = Nodo Inicial
		 L = Nodo Final
		 V = Nodo Visitado
		 N = Nodo No Visitado
	*/

	queue<nodo *> Q;
	//int  hijo;
	nodo *padre, * hijo;
	bool found = false;
	Q.push(&nodes[D]);
	nodes[D].flag = 'V';

	printf("=====================> BUSQUEDA <================== \n");
	while (! Q.empty() && !found) {
		padre = Q.front();
		Q.pop();
		for (size_t i = 0; i < padre->num_conections; i++) {
			hijo = & nodes[padre->conections[i].node]; //Utiliza referencia
			if (hijo->flag == 'N') {
				hijo->flag = 'V';
				hijo->parent = padre->num_node;
				Q.push(hijo);
				printf("\nnodo padre: %d , con hijo: %d", padre->num_node, hijo->num_node);
			}
			if (hijo->num_node == L) {
				found = true;
				break;
			}
		}
	}
}


void A_star_algorithm(int D, int L){
	list<nodo *> open;
	nodo *node_goal, *node_start, *nextNode;
	bool found = false;
	float f, acc_cost;

	node_start = &nodes[D];
	node_goal = &nodes[L];

	open.push_back(node_start);
	node_start->flag = 'V';
	node_start->acumulado = 0.0;
	node_start->heuristica = sqrt(pow(node_start->x - node_goal->x ,2) +
																pow(node_start->y - node_goal->y ,2));

	while (! open.empty() && !found){
		nextNode = getBetterNode(open);
		open.remove(nextNode);
		found = setSuccessors(nextNode, node_goal, open);
	}
}


/*Retorna el nodo con la función de evaluación mejor puntuada (menor)
para toda la lista de nodos abiertos*/
nodo * getBetterNode(list<nodo *> &candidates){//Se recibe referencia
	nodo *best;
	float f, best_f = 1000000.00;
	printf("\n\n======> Lista de nodos:\n");
	list<nodo *>::iterator nodo_it;
  for (nodo_it=candidates.begin(); nodo_it != candidates.end(); ++nodo_it){
		f = (*nodo_it)->heuristica + (*nodo_it)->acumulado;
		printf("---> Nodo %d con un puntuaje %f\n", (* nodo_it)->num_node, f);
		if (f < best_f) {
			best = *nodo_it;
			best_f = f;
		}
	}
	return best;
}

/* Añade a la lista de nodos abiertos los nodos hijos de el nodo padre enviado
como argumento. Los nodos encontrados se actializan con los valores de costo
acumulado y la eurística (distancia real al nodo objetivo)
*/
bool setSuccessors(nodo *padre, nodo *node_goal, list<nodo *> &open){
	nodo *hijo;
	float accNodo;
	printf("\n>>>>>>> Sucesores del nodo: %d \n",padre->num_node );
	for (size_t i = 0; i < padre->num_conections; i++) {
		hijo = & nodes[padre->conections[i].node]; //Utiliza referencia
		accNodo = padre->acumulado + padre->conections[i].cost;
		if (hijo->flag == 'N') {
			hijo->flag = 'V';
			hijo->parent = padre->num_node;
			hijo->heuristica = sqrt(pow(node_goal->x - hijo->x ,2) +
															pow(node_goal->y - hijo->y ,2));
			hijo->acumulado = accNodo;
			open.push_back(hijo);
			printf("\n>>> Nodo hijo encontrado: %d", hijo->num_node);
		}else if(hijo->acumulado > accNodo){ //Cuando vuelve a encontar al mismo nodo
			hijo->parent = padre->num_node;
			hijo->acumulado = accNodo;
			printf("\n>>>> Cambio nodo padre del hijo: %d", hijo->num_node);
		}
	}
	return hijo->num_node == node_goal->num_node;
}
