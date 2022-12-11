
/**
 * \file		simulation.c
 * \version		Rendu 2
 * \date		2018-04-22
 * \author		269739 - 269634
 * \brief		Program for the spring 2018 project of CS-112(c)
 */
 
 //********************************************************************
 //				Inclusion de fichiers en-tête
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <GL/glu.h>

#include "error.h"
#include "constantes.h"
#include "utilitaire.h"
#include "robot.h"
#include "graphic.h"
#include "particule.h"

static int truth = 1;
static int truth2 = 1;
static int l = 0;

//*********************************************************************
//				Ensemble des fonctions du module simulation

void    simulation_tri_robot(int k)
{
	double *ecart_angle1 = NULL;
	double *ecart_angle2 = NULL;
	double dist1,dist2, ka,si , t_min1,t_min2;
	int i, j;
	dist1=0;
	dist2=0;
	ecart_angle1 =&si;
	ecart_angle2 = &ka;
	
	for(i = 0; i < robot_retour_nb_robot(); i++)
	{
		util_ecart_angle(robot_retour_struct_robot(i).centre, robot_retour_alpha(i),
		particule_retour_struct_particules(k).centre, ecart_angle1);
		dist1 = util_distance(robot_retour_struct_robot(i).centre,particule_retour_struct_particules(k).centre);
		t_min1 =( fabs(*ecart_angle1)/(VROT_MAX)) + (dist1/(VTRAN_MAX)) ;		
		for(j = 0; j < robot_retour_nb_robot(); j++)
		{
			util_ecart_angle(robot_retour_struct_robot(j).centre, robot_retour_alpha(j),
			particule_retour_struct_particules(k).centre, ecart_angle2);
			dist2=util_distance(robot_retour_struct_robot(j).centre,particule_retour_struct_particules(k).centre);
			t_min2 = (fabs(*ecart_angle2)/(VROT_MAX)) + (dist2/(VTRAN_MAX));
			if((t_min2 < t_min1 && j > i) && robot_etat_robot(i) == 0 && robot_retour_selec(j)==0 )
			{
				robot_remplace_robot(i,j);		
			}
		}
	}
	robot_change_etat1(k);
	l++;
	if( l == 6)
		l = 0;
}

int simulation_live_collision_R_P(int i, double DELTA_D, double vrot)
{
	int collision = 0;
	unsigned int j ;
	double p_dist;
	int nb_particules_sim;
	C2D struct_par;
	double alignement;
	double vrot2, L, la_new, vtran;
	int h;
	nb_particules_sim = particule_retour_nb_particules();
	robot_change_p(i);
	robot_p_new(vrot, DELTA_D);
	for( j = 0; j < nb_particules_sim; j++)
	{
		if(particule_existe(j) && collision == 0)
		{
			struct_par = particule_retour_struct_particules(j);
			L = util_distance(robot_retour_struct_robot(i).centre, struct_par.centre);
			//printf("voila %d %lf %lf %lf %lf\n", ver, D, L, vrot, DELTA_D);
			if(util_collision_cercle(struct_par, robot_return_p(), &p_dist) && 
					util_inner_triangle(fabs(DELTA_D), p_dist, L,
								   (R_ROBOT+struct_par.rayon), &la_new))
			{
				//printf("brika\n");
				vtran = (la_new/DELTA_T);
				if (vtran >VTRAN_MAX )	
					vtran = VTRAN_MAX;	
				else if (vtran < -VTRAN_MAX )
					vtran = -VTRAN_MAX;
				la_new = vtran*DELTA_T;
				robot_new_coo(i, 0, la_new);
				util_ecart_angle(robot_return_p().centre,
				robot_return_alpha(),struct_par.centre, &alignement);
				h = util_alignement(robot_return_p().centre, 
						robot_return_alpha(), struct_par.centre);
				if(h)
				{
					particule_detruit(j);
					robot_change_etat0(i);
					simulation_tri_robot(j);
				}
				else
				{
					vrot2= (alignement/DELTA_T);
					if (vrot2 > VROT_MAX)
						vrot2 = VROT_MAX;
					else if (vrot2 < -VROT_MAX)
						vrot2 = -VROT_MAX; 
					robot_new_coo(i, vrot2, 0);
					collision = 1;
				}
			}
			else if(util_collision_cercle(struct_par, robot_retour_struct_robot(i), &p_dist)  
						)
			{
				//printf("AHLA HABIBI\n");
				util_ecart_angle(robot_retour_struct_robot(i).centre,
				robot_retour_alpha(i),struct_par.centre, &alignement);
				h = util_alignement(robot_retour_struct_robot(i).centre, 
						robot_retour_alpha(i), struct_par.centre);
				if(h)
				{
					robot_change_etat0(i);
					particule_detruit(j);
				}
				else
				{ 
					vrot2= (alignement/DELTA_T);
					if (vrot2 > VROT_MAX)
						vrot2 = VROT_MAX;
					else if (vrot2 < -VROT_MAX)
						vrot2 = -VROT_MAX; 
					robot_new_coo(i, vrot2, 0);
					collision = 1;
				}
			}
			else if(util_collision_cercle(struct_par, robot_return_p(), &p_dist) && 
					!util_inner_triangle(fabs(DELTA_D), p_dist, L,
								   (R_ROBOT+struct_par.rayon), &la_new))
			{
				
				util_ecart_angle(robot_return_p().centre,
				robot_return_alpha(),struct_par.centre, &alignement);
				h = util_alignement(robot_return_p().centre, 
						robot_return_alpha(), struct_par.centre);
				if(h)
				{
					particule_detruit(j);
					robot_change_etat0(i);
					simulation_tri_robot(j);
				}
				else
				{
					vrot2= (alignement/DELTA_T);
					if (vrot2 > VROT_MAX)
						vrot2 = VROT_MAX;
					else if (vrot2 < -VROT_MAX)
						vrot2 = -VROT_MAX; 
					robot_new_coo(i, vrot2, 0);
					collision = 1;
				}
			}
		}
	}
	return collision;
}

/*int simulation_live_collision_R_P(int i)
{
	int collision = 0;
	unsigned int j ;
	double p_dist;
	int nb_particules_sim;
	C2D struct_rob, struct_par ;
	double alignement;
	double vrot;
	int h;
	nb_particules_sim = particule_retour_nb_particules();
	struct_rob = robot_retour_struct_robot(i);
	for( j = 0; j < nb_particules_sim; j++)
	{
		if(particule_existe(j))
		{
			struct_par = particule_retour_struct_particules(j);
			if(util_collision_cercle(struct_par, struct_rob, &p_dist))
			{
				util_ecart_angle(struct_rob.centre, robot_retour_alpha(i),
										struct_par.centre, &alignement);
				h = util_alignement(struct_rob.centre, 
						  robot_retour_alpha(i), struct_par.centre);
				if(h)
				{
					robot_change_etat0(i);
					particule_detruit(j);
				}
				else
				{
					vrot= alignement/DELTA_T;
					if (vrot > VROT_MAX)
						vrot = VROT_MAX;
					else if (vrot < -VROT_MAX)
						vrot = -VROT_MAX; 
					robot_new_coo(i, vrot, 0);
					collision = 1;
				}
			}
		}
	}
	if(collision)
		return 1;
	else
		return 0;
}*/



/*void simulation_tri_occupation()
{
	int i, j;
	for(i = 0; i < robot_retour_nb_robot(); i++)
	{
		for(j = 0; j < robot_retour_nb_robot(); j++)
		{
			if(robot_etat_robot(j) < robot_etat_robot(i) && j > i)
				robot_remplace_robot(i, j);
		}
	}
	
}*/

void simulation_init_robot()
{	
	int i;
	for (i=0 ;i<robot_retour_nb_robot(); i++)
		robot_change_etat0(i);
}
void simulation_init_selec()
{
	int i ;
	for (i=0 ;i<robot_retour_nb_robot(); i++)
		robot_change_selec0(i);
	
}

int simulation_test_collision_R_P()
{
	unsigned int i,j ;
	double p_dist;
	int nb_robot_sim;
	int nb_particules_sim;
	C2D struct_rob ;
	C2D struct_par ;
	
	nb_robot_sim = robot_retour_nb_robot();
	nb_particules_sim = particule_retour_nb_particules();
	
	for( i = 0; i < nb_robot_sim ; i++)
	{		
		struct_rob = robot_retour_struct_robot(i);
		for( j = 0; j < nb_particules_sim; j++)
		{
			struct_par = particule_retour_struct_particules(j);
			if(j != i && 
			   util_collision_cercle(struct_par, struct_rob, &p_dist))
			{
				error_collision(ROBOT_PARTICULE, (j+1), (i+1));
				return 0;
			}
		}
	}
	return 1;
}

int simulation_lecture_fichier(FILE* fichier)
{
	int line_number;
	long int curseur = 0;
	if(robot_lecture_fichier( fichier))
	{
		curseur = robot_get_curseur(fichier);
		line_number = robot_get_line_number();
		if(particule_lecture_fichier(fichier, curseur, line_number))
		{	
			if(simulation_test_collision_R_P())
				{
					error_no_error_in_this_file();
					return 1;
				}
			else
				return 0;
		}
		else
			return 0;
	}	
	else
		return 0;
}


void simulation_dessine()
{	int i ;
	glColor3f(0.5, 0.5, 0.5);
	particule_dessine_initiale();
	for (i=0 ; i< robot_retour_nb_robot();i++)
	{
		if (robot_retour_selec(i)==0)
		{
			glColor3f(0.0,0.0,0.0);
			robot_dessine_robot(i);
		}
		else if(robot_retour_selec(i)==1)
		{
			glColor3f(1.0,0.0,0.0);
			robot_dessine_robot(i);
			
		}
		
	}
}


void simulation_init_decomposition()
{
	particule_decomposition1();
}

void simulation_move_robot()
{
	double vrot, vtran, DELTA_D, D;
	int i;  
	double *ecart_angle = NULL;
	double ka;
	ecart_angle = &ka;
	int nb_robot = robot_retour_nb_robot() ;
	if(truth)
	{
		for(i = 0; i < nb_robot; i++)
			simulation_init_robot(i);
	}
	truth = 0;
	truth2 = 0;
	for ( i = 0; i < nb_robot; i++)
	{	
		if(particule_existe(i) && robot_retour_selec(i)==0)
		{
		//simulation_tri_occupation();
			robot_change_etat1(i);	
		//	util_collision_cercle(robot_retour_struct_robot(i), particule_retour_struct_particules(i) ,k);
			util_ecart_angle(robot_retour_struct_robot(i).centre, robot_retour_alpha(i),
			particule_retour_struct_particules(i).centre, ecart_angle);
			if (fabs(*ecart_angle) >  (M_PI/2)) 
				vtran = 0;
			else if (fabs(*ecart_angle)<(M_PI/2)) 
			{
				D = util_distance(robot_retour_struct_robot(i).centre,
				particule_retour_struct_particules(i).centre);
				if( D < (R_ROBOT + particule_retour_struct_particules(i).rayon - EPSIL_ZERO)) 
					D = 0;
				vtran = (D/DELTA_T);
				if (vtran >VTRAN_MAX )	
					vtran = VTRAN_MAX;	
				else if (vtran < -VTRAN_MAX )
					vtran = -VTRAN_MAX;
			}
			DELTA_D = vtran * DELTA_T;
			vrot= *ecart_angle/DELTA_T;
			if (vrot > VROT_MAX)
				vrot = VROT_MAX;
			else if (vrot < -VROT_MAX)
				vrot = -VROT_MAX;
			if(!robot_live_collisionR_R(DELTA_D, i, vrot))
			{
				if(!simulation_live_collision_R_P(i, DELTA_D, vrot))
					robot_new_coo(i,vrot,DELTA_D);
			}		
		}
	}
}

void simulation_save_current_state(FILE* save)
{
	robot_save_current_state(save);
	particule_save_current_state(save);
}

void simulation_free_and_clear()
{
	particule_free();
	robot_free();
}

double simulation_decontamination()
{
	return particule_decontamination();
}
void simulation_verif_souris(double x,double y )
{
	int i  ;
	
	S2D souris;
	souris.x = x; 
	souris.y = y;
	printf("%f %f",x,y);
	simulation_init_selec();
	for (i=0 ; i<robot_retour_nb_robot() ; i++)
	{
		printf("robot %d ahawa %d \n",i,robot_retour_selec(i));
		if (util_point_dans_cercle(souris , robot_retour_struct_robot(i))==1)
		{
			robot_change_selec1(i);
			printf("robot %d ahawa %d \n",i,robot_retour_selec(i));
		}
		robot_change_etat0(i);	
	}	
}

void simulation_robot_new_coo(double vrota,double vtrans)
{
	double DELTA_D;
	int i;
	for (i=0;i<robot_retour_nb_robot();i++)
	{
	if (robot_retour_selec(i)==1)
	{
		
		DELTA_D= vtrans * DELTA_T;
		robot_new_coo(i,vrota,DELTA_D);
	}
	}
}


