#include "Compare_Test_2.h"
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <windows.h>
#include <random>

using namespace std;
/*
void generate_SNs_PSO(SENSOR_NODE s[])
{

	int i,k;
	bool flag;
	//srand(time(NULL));
	for(i=0;i<SN_NUM;i++)
	{
		s[i].id = i;
		do{
			s[i].pos.x = rand()%(X - 2*(COMM_RANGE))+COMM_RANGE;
			s[i].pos.y = rand()%(Y - 2*(COMM_RANGE))+COMM_RANGE;
			s[i].pos.z = 0;
			flag = false;
			for(k=0;k<i;k++)
			{
				if(dist(s[i].pos,s[k].pos)<COMM_RANGE)
				{
					flag = true;
					break;
				}
			}
		}while(flag);
	}
}
*/
/*
double dist_PSO(POS a,POS b)
{
	double sum = (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
	return sqrt(sum);
}
*/

double setFitness(Individual& solution,SENSOR_NODE s[])
{
	set<short> entire_sensor_nodes;
	entire_sensor_nodes.clear();
	for(short i=0;i<SN_NUM;i++)
		entire_sensor_nodes.insert(i);
	vector<WAYPOINT> path = solution.path;
	float tour_length = 0.0;
	float col_time = 0.0;
	float Ef;
	float Ec;
	double fitness;
	set<short>::iterator iter_sensor_nodes;
	//calculate the total length and flight energy.
	for(unsigned int i=0;i<path.size()-1;i++)
	{
		tour_length = tour_length + dist_PSO(path[i].pos,path[i+1].pos);
	}
	tour_length = tour_length + dist_PSO(path[path.size()-1].pos,path[0].pos);
	Ef = E1 * tour_length / velocity;
	//calculate the time and the energy spent on collecting data
	for(unsigned int i=1; i<path.size(); i++)
	{
		iter_sensor_nodes =  path[i].ss.begin();
		while(iter_sensor_nodes!=path[i].ss.end())
		{
			short j = *iter_sensor_nodes;
			if(entire_sensor_nodes.find(j) != entire_sensor_nodes.end())
			{
				float dateRate = getDataRatePSO(path[i].pos,s[j].pos);
				col_time = col_time + sensorData * 8 /dateRate;
				entire_sensor_nodes.erase(j);
			}
			iter_sensor_nodes++;
		}
	}
	Ec = E2 * col_time;
	fitness = (Ef + Ec);
	solution.tour_length = tour_length;
	solution.collect_time = col_time;
	solution.fitness = fitness;
	return fitness;
}

void setPSOFitness(Particle &particle)
{
	vector<POS> path = particle.position;
	double tour_length = 0.0;

	for(unsigned int i=0;i<path.size()-1;i++)
	{
		tour_length = tour_length + dist_PSO(path[i],path[i+1]);
	}
	tour_length = tour_length + dist_PSO(path[path.size()-1],path[0]);
	particle.fitness = tour_length;

}

bool isFeasiblePath(vector<POS> &path,SENSOR_NODE s[])
{
	int i;
	bool flag1 = false;
	bool flag2 = false;
	for( i=0;i<SN_NUM;i++)
	{
		vector<POS>::iterator iter_path  = path.begin();
		while(iter_path != path.end())
		{
			if(dist_PSO(*iter_path,s[i].pos)<=COMM_RANGE)
			{
				break;
			}
			iter_path++;
		}
		if(iter_path == path.end())
			break;
	}
	if(i== SN_NUM)
		flag1 = true;

	vector<POS>::iterator iter_path  = path.begin();
	while(iter_path != path.end())
	{
		if((*iter_path).x < 0 ||(*iter_path).x > X ||(*iter_path).y < 0 ||(*iter_path).y > Y )
		{
			break;
		}
		iter_path++;
	}

	if(iter_path == path.end())
		flag2 = true;
	if(flag1 && flag2)
		return true;
	else
		return false;
}


void generateParticles(SENSOR_NODE s[])
{
	swarm.clear();
	Particle temp;
	//initial the start point
	POS start;
	start.x = x_start;
	start.y = y_start;
	start.z = he;
	srand(time(NULL));
	for(int i=0;i<particle_num;)
	{
		int num_dimen = SN_NUM;//rand()%SN_NUM + SN_NUM ;
		temp.position.clear();
		temp.position.push_back(start);
		temp.fitness = 0.0;
		temp.pbest_fitness =0.0;

		for(int i=1;i<num_dimen;i++)
		{
			POS pos;
			pos.x = rand()%X;
			pos.y = rand()%Y;
			pos.z = he;
			temp.position.push_back(pos);
		}
		bool flag = isFeasiblePath(temp.position,s);
		if(flag)
		{
			setPSOFitness(temp);
			temp.pbest.clear();
			temp.v.clear();
			temp.pbest = temp.position;
			temp.pbest_fitness = temp.fitness;

			temp.v.push_back(start);
			for(int i=1;i<num_dimen;i++)
			{
				POS pos;
				pos.x = rand()%COMM_RANGE;
				pos.y = rand()%COMM_RANGE;
				pos.z = he;
				//Sleep(1000);
				temp.v.push_back(pos);
			}

			swarm.push_back(temp);
			i++;
			cout<<i<<endl;
			Sleep(1000);
		}
	}

}

bool lessSortParticle(Particle a,Particle b){return a.fitness < b.fitness;}

Particle sortParticles(vector<Particle>& particles)
{
	vector<Particle>::iterator be = particles.begin();
	vector<Particle>::iterator en = particles.end();
	sort(be,en,lessSortParticle);
	return particles.at(0);
}

void reverseOrder(Particle &particle,double reverse_probability)
{
	/*
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);
	double random_prob = dis(gen);
	if(random_prob > reverse_probability)
	{
	*/
		int i=0,j=0;
		//srand(time(NULL));
		i = rand()%(particle.position.size()-2)+1;
		j = rand()%(particle.position.size()-i-1) + (i+1);
		
	
		for(int k=0;k<j-i;k++)
		{
			POS temp = particle.position.at(j);
			particle.position.erase(particle.position.begin()+j);
			particle.position.insert(particle.position.begin()+i+k,temp);
		}
		setPSOFitness(particle);
	//}
}

void mergeRPs(Particle &particle)
{
	vector<POS> temp_path ;
	temp_path.clear();
	temp_path = particle.position;
	vector<POS>::iterator iter_temp_path =  temp_path.begin()+1;
	for(;iter_temp_path!=temp_path.end()-1;iter_temp_path++)
	{
		if(dist_PSO(*iter_temp_path,*(iter_temp_path+1)) < (0.25 * COMM_RANGE))
		{
			POS temp_pos = (*iter_temp_path) + *(iter_temp_path+1);
			temp_pos = temp_pos/2;
			iter_temp_path = temp_path.erase(iter_temp_path);
			iter_temp_path = temp_path.erase(iter_temp_path);
			temp_path.insert(iter_temp_path,temp_pos);
			--iter_temp_path;

		}
	}
	particle.position.clear();
	particle.position = temp_path;
	setPSOFitness(particle);
}

POS findNearestDimen(const POS &pos,vector<POS> &path)
{
	double min_len = INT_MAX;
	vector<POS>::iterator iter_path = path.begin();
	POS res = (*iter_path);
	for(;iter_path != path.end();iter_path++)
	{
		if(dist_PSO(pos,(*iter_path)) < min_len)
		{
			min_len = dist_PSO(pos,(*iter_path));
			res = (*iter_path);
		}
	}
	return res;
}

void updateParticles(Particle &particle,vector<POS> &gbest)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);

	POS pbest_nearest_dimen;
	POS gbest_nearest_dimen;
	vector<POS> temp_v;
	temp_v.clear();
	temp_v	= particle.v;

	double r1;
	double r2;
	for(int i=1;i<particle.position.size();i++)
	{
		r1 = dis(gen);
		r2 = dis(gen);
		//cout << r1 << " "<<r2<<endl;
		pbest_nearest_dimen = findNearestDimen(particle.position[i],particle.pbest);
		gbest_nearest_dimen = findNearestDimen(particle.position[i],gbest);
		particle.v[i] =  particle.v[i] * wei + (pbest_nearest_dimen - particle.position[i]) * (c1 * r1)+ (gbest_nearest_dimen - particle.position[i]) * (c2 * r2) ;
		particle.position[i] = particle.position[i] + temp_v[i];
	}

	setPSOFitness(particle);
}

void simulatedAnnealing(Particle &particle,vector<POS> &oldPath,double old_fitness,double temperature,SENSOR_NODE s[])
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 1);
	double prob;
	double random_prob;
	if(isFeasiblePath(particle.position,s))
	{
		if(particle.fitness >= old_fitness)
		{
			random_prob = dis(gen);
			double temp = (particle.fitness - old_fitness)/temperature;
			//cout<<particle.fitness<<" "<<old_fitness<<endl;
			//cout<<temperature<<" "<<temp<<endl;
			prob = 1/exp(temp);
			//cout<<prob <<" "<<random_prob<<endl;
			if(random_prob > prob)
			{
				particle.position = oldPath;
				particle.fitness = old_fitness;
			}

		}
		else
		{
			//if(particle.fitness < particle.pbest_fitness)
			//{
				particle.pbest.clear();
				particle.pbest = particle.position;
				particle.pbest_fitness = particle.fitness;
			//}
		}
	}
	else
	{
		particle.position = oldPath;
		particle.fitness = old_fitness;
	}
}

Particle VD_PSO(SENSOR_NODE s[])
{
	vector<POS> gbest;
	gbest.clear();
	vector<Particle> temp = swarm;
	gbest = sortParticles(temp).position;
	double temperature = 100.0;
	double reverse_prob = 0.5;

	vector<POS> path_temp;
	double fitness_temp;
	for(int i=0;i<generations;i++)
	{
		for(int j=0;j<particle_num;j++)
		{
			path_temp.clear();
			path_temp = swarm.at(j).position;
			fitness_temp = swarm.at(j).fitness;
			updateParticles(swarm.at(j),gbest);
			simulatedAnnealing(swarm.at(j),path_temp,fitness_temp,temperature,s);

			path_temp.clear();
			path_temp = swarm.at(j).position;
			fitness_temp = swarm.at(j).fitness;
			mergeRPs(swarm.at(j));
			simulatedAnnealing(swarm.at(j),path_temp,fitness_temp,temperature,s);

			path_temp.clear();
			path_temp = swarm.at(j).position;
			fitness_temp = swarm.at(j).fitness;
			reverseOrder(swarm.at(j),reverse_prob);
			simulatedAnnealing(swarm.at(j),path_temp,fitness_temp,temperature,s);

		}
		temp = swarm;
		gbest.clear();
		gbest = sortParticles(temp).position;

		temperature = temperature * attenuation_factor;
		cout<<i<<" "<<temp[0].position.size()<<" "<<temp[0].fitness<<endl;
		/*
		vector<POS>::iterator iter_path = gbest.begin();
		while(iter_path != gbest.end())
		{
			cout<<(*iter_path).x<<" "<<(*iter_path).y<<" "<<(*iter_path).z<<" "<<"||"<<" ";
			iter_path++;
		}
		cout<<endl;
		*/
	}
	sortParticles(swarm);
	return swarm.at(0);
}

Individual testAlgorithm2(SENSOR_NODE s[])
{
	Individual solu;
	Particle p;
	WAYPOINT wp;
	vector<POS> path;

	vector<POS>::iterator iter_path;
	set<short> sensors;

	sensors.clear();
	for(short i=0;i<SN_NUM;i++)
		sensors.insert(i);

	generateParticles(s);
	p = VD_PSO(s);
	path = p.position;
	iter_path = path.begin();
	while(iter_path != path.end())
	{
		//cout<<(*iter_path).x<<" "<<(*iter_path).y<<" "<<(*iter_path).z<<" "<<endl;
		wp.pos = *iter_path;
		wp.ss.clear();
		for(int i=0;i<SN_NUM;i++)
		{
			if((dist_PSO((*iter_path),s[i].pos) <= COMM_RANGE) && (sensors.find(i)!= sensors.end()))
			{
				wp.ss.insert(s[i].id);
				sensors.erase(i);
			}
		}
		solu.path.push_back(wp);
		iter_path++;
	}
	setFitness(solu,s);

	return solu;
}