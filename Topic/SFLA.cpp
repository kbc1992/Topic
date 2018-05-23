#include "SFLA.h"
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <windows.h>
#include <fstream>
#include <iterator>
#include <cstdio>

using namespace std;

double getFitness(Individual& solution,SENSOR_NODE s[])
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
        tour_length = tour_length + dist(path[i].pos,path[i+1].pos);
    }
    tour_length = tour_length + dist(path[path.size()-1].pos,path[0].pos);
    Ef = E1 * tour_length / velocity;
    //calculate the time and the energy spent on collecting data
    for(unsigned int i=1; i<path.size()-1; i++)
    {
        iter_sensor_nodes =  path[i].ss.begin();
        while(iter_sensor_nodes!=path[i].ss.end())
        {
            short j = *iter_sensor_nodes;
            if(entire_sensor_nodes.find(j) != entire_sensor_nodes.end())
            {
                float dateRate = getDataRate(point_to_int(path[i].pos),s[j].pos);
                col_time = col_time + sensorData * 8 / dateRate;
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

Individual randGeneOneSolut(map<set<short>,vector<int> >& optimal_overlapping_regions,SENSOR_NODE s[],short h)
{
    Individual solution;
    //randomly decide the traversal order
    vector<set<short> > vec_overlap_region;
    vec_overlap_region.clear();
    map<set<short>,vector<int> >::iterator iter_optimal_overlapping_regions = optimal_overlapping_regions.begin();
    while(iter_optimal_overlapping_regions != optimal_overlapping_regions.end())
    {
        vec_overlap_region.push_back(iter_optimal_overlapping_regions->first);
        iter_optimal_overlapping_regions++;
    }

    //initial the start point
    WAYPOINT start;
    start.pos.x = x_start;
    start.pos.y = y_start;
    start.pos.z = h;
    solution.path.push_back(start);
    //randomly generate a point from the chosen region.
    srand(time(NULL));
    int num_col_point = vec_overlap_region.size();
    for(int i=0;i<num_col_point;i++)
    {
        int j = rand()%vec_overlap_region.size();
        WAYPOINT wp;
        wp.ss = vec_overlap_region[j];
        int k = rand()%optimal_overlapping_regions.find(vec_overlap_region[j])->second.size();
        POS pos;
        int_to_point(optimal_overlapping_regions.find(vec_overlap_region[j])->second.at(k),pos);
        wp.pos = pos;
        solution.path.push_back(wp);
        vec_overlap_region.erase(vec_overlap_region.begin()+j);
    }
    solution.fitness = getFitness(solution,s);
    return solution;
}


Individual geneOneSolutByBestIndi(Individual& bestSolution,map<set<short>,vector<int> >& optimal_overlapping_regions,SENSOR_NODE s[],short h)
{
	Individual solution;
	vector<WAYPOINT>::iterator iter_path = bestSolution.path.begin() + 1;
	srand(time(NULL));
	WAYPOINT start;
    start.pos.x = x_start;
    start.pos.y = y_start;
    start.pos.z = h;
    solution.path.push_back(start);
	while(iter_path != bestSolution.path.end())
	{
		WAYPOINT wp;
		set<short> temp_sens = (*iter_path).ss;
		wp.ss = temp_sens;
		vector<int> temp_points = optimal_overlapping_regions.find(temp_sens)->second;
		int k = rand()%temp_points.size();
		POS pos;
		int_to_point(temp_points.at(k),pos);
		wp.pos = pos;
		solution.path.push_back(wp);
		iter_path++;
	}
	solution.fitness = getFitness(solution,s);
	return solution;
}

bool isFeasibleSolution(Individual& solution)
{
    set<short> entire_sensor_nodes;
    set<short> sensor_nodes_path_contained;
    sensor_nodes_path_contained.clear();
    entire_sensor_nodes.clear();
	bool flag1 = true;
	bool flag2 = true;
    for(short i=0; i<SN_NUM; i++)
        entire_sensor_nodes.insert(i);
    vector<WAYPOINT> path = solution.path;
    for(vector<WAYPOINT>::iterator iter_path = path.begin()+1;iter_path != path.end();iter_path++)
    {
        set<short> temp;
		POS temp_pos;
        temp.clear();
        set_union(sensor_nodes_path_contained.begin(),sensor_nodes_path_contained.end(),(*iter_path).ss.begin(),(*iter_path).ss.end(),inserter(temp,temp.begin()));
        sensor_nodes_path_contained = temp;
		temp_pos = (*iter_path).pos;
		if(temp_pos.x < 0 || temp_pos.y < 0 || temp_pos.z < minimal_height/CUBE_LEN)
		{
			flag1 = false;
			break;
		}
    }

    if(sensor_nodes_path_contained != entire_sensor_nodes )
        flag2 = false;

	if(flag1 && flag2)
		return true;
	else
		return false;
    
}

void init(map<set<short>,vector<int> >& optimal_overlapping_regions,vector<Individual>& population,SENSOR_NODE s[],short h)
{
	Individual temp;
    for(int i=0; i<Popus; i++)
    {
        temp = randGeneOneSolut(optimal_overlapping_regions,s,h);
        population.push_back(temp);

        /*
        //if(i<20)
        //{
            vector<WAYPOINT> path_temp = temp.path;
            vector<WAYPOINT>::iterator iter_path_temp = path_temp.begin();
            while(iter_path_temp != path_temp.end())
            {
                cout<<(*iter_path_temp).pos.x<<" "<<(*iter_path_temp).pos.y<<" "<<(*iter_path_temp).pos.z<<endl;
                iter_path_temp++;
            }
        //}
        */
        Sleep(800);

    }

}

bool lessSort(Individual a,Individual b){return a.fitness < b.fitness;}

void sortIndividuals(vector<Individual>& population)
{
	DWORD start, stop;  
	//start = GetTickCount(); 


    vector<Individual>::iterator be = population.begin();
    vector<Individual>::iterator en = population.end();
    sort(be,en,lessSort);

	//stop = GetTickCount();  
	//printf("the time to sort populations:: %d ms\n", stop - start);  
}

void seperateIntoSubPopulation(vector<Individual>& population,vector<vector<Individual> >& subPopulations)
{
    for(int i=0;i<Meme;i++)
    {
        vector<Individual> temp;
        temp.clear();
        for(int j=0;j<Indis;j++)
        {
            temp.push_back(population[j*Meme+i]);
        }
        subPopulations.push_back(temp);
    }
}

void mergyIntoPopulation(vector<Individual>& population,vector<vector<Individual> >& subPopulations)
{
    population.clear();
    for(int i=0;i<Meme;i++)
    {
        for(int j=0;j<Indis;j++)
        {
            population.push_back(subPopulations[i][j]);
        }
    }
}

void constructGroup(vector<Individual>& subPopulation,vector<Individual>& subGroup)
{
    float prob_chosen[Indis];
    int i;
    float rand_prob;
    sortIndividuals(subPopulation);
    for(i=0;i<Indis;i++)
    {
        prob_chosen[i] = (2*(Indis-i))/((Indis-1)*Indis);
    }
    i=0;
    srand(time(NULL));
    while(i!=GInd)
    {
        rand_prob = 1/((rand()%((Indis-1)*(Indis-1)/2 + 1)) + (Indis-1)/2);
        while(true)
        {
            int j = rand()%subPopulation.size();
            if(prob_chosen[j]>=rand_prob)
            {
                subGroup.push_back(subPopulation[j]);
				subPopulation.erase(subPopulation.begin()+j);
                break;
            }
        }
        i++;
    }
}

void reBuildSubPopulation(vector<Individual>& subPopulation,vector<Individual>& subGroup)
{
    subPopulation.insert(subPopulation.end(),subGroup.begin(),subGroup.end());
}

void traversalOrderUpdate(Individual& bestSolution,Individual& worstSolution,SENSOR_NODE s[])
{
	DWORD start, stop;  
	//start = GetTickCount(); 

    srand(time(NULL));
    float rand_step_size;
    vector<POS> step_size;
    vector<WAYPOINT>::iterator iter_best_solu = bestSolution.path.begin() + 1;
    vector<WAYPOINT>::iterator iter_worst_solu = worstSolution.path.begin() + 1;
    while(iter_best_solu != bestSolution.path.end())
    {
		rand_step_size = (float)(rand()%20)/100 + 0.9;
		/*
        (*iter_worst_solu).pos.x += ((*iter_best_solu).pos.x -(*iter_worst_solu).pos.x) * rand_step_size;
        (*iter_worst_solu).pos.y += ((*iter_best_solu).pos.y -(*iter_worst_solu).pos.y) * rand_step_size;
        (*iter_worst_solu).pos.z += ((*iter_best_solu).pos.z -(*iter_worst_solu).pos.z) * rand_step_size;
        */
		(*iter_worst_solu).pos = (*iter_worst_solu).pos + ((*iter_best_solu).pos -(*iter_worst_solu).pos) * rand_step_size;
		(*iter_worst_solu).ss.clear();
        set<short>::iterator iter_sensor_nodes = (*iter_best_solu).ss.begin();
        while(iter_sensor_nodes!=(*iter_best_solu).ss.end())
        {
            if(dist((*iter_worst_solu).pos,s[(*iter_sensor_nodes)].pos) <= COMM_RANGE)
            {
                (*iter_worst_solu).ss.insert(*iter_sensor_nodes);
            }
            iter_sensor_nodes++;
        }
        iter_best_solu++;
        iter_worst_solu++;
    }
	getFitness(worstSolution,s);

	//stop = GetTickCount();  
	//printf("the time to update traversal order: %d ms\n", stop - start);  
}

void convertToPos(vector<int>& points,vector<POS>& pos_points)
{
    vector<int>::iterator iter_points = points.begin();
    POS temp;
    while(iter_points != points.end())
    {
        int_to_point(*iter_points,temp);
        pos_points.push_back(temp);
        iter_points++;
    }

}
/*
POS findBestDCP(vector<int>& points)
{
    vector<POS> pos_points;
    double radius = 1e20; //答案=最小覆盖球的半径
    double delta = 100; //每次移动的变化量
    POS start;
    convertToPos(points,pos_points);
    start = pos_points[0];
    while(delta>EPS)
    {
        int d = 0; //d为离当前确定的圆心最远的点的编号
        for(unsigned int i=0;i<pos_points.size();i++)
            if(dist(start,pos_points[i])>dist(start,pos_points[d]))
                d=i;
        double nowr=dist(start,pos_points[d]); //nowr=当前固定圆心的最小覆盖球的半径大小
        radius = min(radius,nowr);
        start.x+=(short)((pos_points[d].x-start.x)/nowr*delta);
        start.y+=(short)((pos_points[d].y-start.y)/nowr*delta);
        start.z+=(short)((pos_points[d].z-start.z)/nowr*delta);
        delta*=0.98;
    }
    return start;
}
*/

POS findBestDCP(vector<int>& points)
{
	int x_sum = 0;
	int y_sum = 0;
	int z_sum = 0;
	POS center;
	vector<POS> pos_points;
	pos_points.clear();
	convertToPos(points,pos_points);

	for(int i=0;i<pos_points.size();i++)
	{
		x_sum = x_sum + pos_points[i].x;
		y_sum = y_sum + pos_points[i].y;
		z_sum = z_sum + pos_points[i].z;
	}
	center.x = (short)x_sum/(pos_points.size());
	center.y = (short)y_sum/(pos_points.size());
	center.z = (short)z_sum/(pos_points.size());

	return center;
}

POS findBestDCPSet(set<short>& overlapping_region,vector<int>& points,SENSOR_NODE s[])
{
    map<float,vector<int> > col_time_points;
	map<float,vector<int> >::iterator iter_col_time_points;
    vector<int> temp_points;
    vector<int>::iterator iter_points = points.begin();
    set<short>::iterator iter_over_reg;
    while(iter_points != points.end())
    {
        temp_points.clear();
        temp_points.push_back(*iter_points);
        float sum_col_time = 0.0;
        iter_over_reg = overlapping_region.begin();
        while(iter_over_reg != overlapping_region.end())
        {
			int temp = *iter_over_reg;
            float date_rate = getDataRate(*iter_points,s[temp].pos);
            float col_time = sensorData * 8/date_rate;
            sum_col_time = sum_col_time + col_time;
			iter_over_reg++;
        }
        pair<map<float,vector<int> >::iterator,bool> ret = col_time_points.insert(pair<float,vector<int> >(sum_col_time,temp_points));
        if(!ret.second)
        {
            ret.first->second.push_back(*iter_points);
        }
        iter_points++;
    }
	iter_col_time_points = --col_time_points.end();
    temp_points = iter_col_time_points->second;
    return findBestDCP(temp_points);
}
/*
POS findCenterAllRegion(SENSOR_NODE s[])
{
    vector<POS> pos_points;
    for(int i=0;i<SN_NUM;i++)
    {
        pos_points.push_back(s[i].pos);
    }
    double radius = 1e20; //答案=最小覆盖球的半径
    double delta = 100; //每次移动的变化量
    POS start;
    start = pos_points[0];
    while(delta>EPS)
    {
        int d = 0; //d为离当前确定的圆心最远的点的编号
        for(unsigned int i=0;i<pos_points.size();i++)
            if(dist(start,pos_points[i])>dist(start,pos_points[d]))
                d=i;
        double nowr=dist(start,pos_points[d]); //nowr=当前固定圆心的最小覆盖球的半径大小
        radius = min(radius,nowr);
        start.x+=(short)((pos_points[d].x-start.x)/nowr*delta);
        start.y+=(short)((pos_points[d].y-start.y)/nowr*delta);
        start.z+=(short)((pos_points[d].z-start.z)/nowr*delta);
        delta*=0.98;
    }
    return start;
}
*/

POS findCenterAllRegion(SENSOR_NODE s[],short h)
{
	int x_sum=0;
	int y_sum=0;
	POS center;
	for(int i=0;i<SN_NUM;i++)
	{
		x_sum = x_sum + s[i].pos.x;
		y_sum = y_sum + s[i].pos.y;
	}
	center.x = (short)x_sum/(SN_NUM+1);
	center.y = (short)y_sum/(SN_NUM+1);
	center.z = h;

	return center;
}

void positionUpdate(Individual& bestSolution,Individual& worstSolution,map<set<short>,vector<int> >& overlapping_regions,SENSOR_NODE s[])
{
	DWORD start, stop;   
	//start = GetTickCount(); 

	srand(time(NULL));
    vector<WAYPOINT>::iterator iter_worstSolu = worstSolution.path.begin()+1;
    vector<WAYPOINT>::iterator iter_bestSolu = bestSolution.path.begin()+1;
    POS center_best_DCP;
    //POS center_all_regions = findCenterAllRegion(s);
    float c1,c2,c3;
    while(iter_worstSolu != worstSolution.path.end())
    {
		set<short> temp_sens = (*iter_worstSolu).ss;
		//vector<int> temp_points = overlapping_regions.find(temp_sens)->second;
        //center_best_DCP = findBestDCPSet(temp_sens,temp_points,s);
		center_best_DCP = center_in_each_region.find(temp_sens)->second;
        c1 = rand()%100/(float)100;
        c2 = rand()%200/(float)X;
        c3 = rand()%100/(float)100;
		/*
        (*iter_worstSolu).pos.x = (*iter_worstSolu).pos.x + c1*((*iter_bestSolu).pos.x-(*iter_worstSolu).pos.x)+c2*(center_all_regions.x - (*iter_worstSolu).pos.x)+c3*(center_best_DCP.x - (*iter_worstSolu).pos.x);
        (*iter_worstSolu).pos.y = (*iter_worstSolu).pos.y + c1*((*iter_bestSolu).pos.y-(*iter_worstSolu).pos.y)+c2*(center_all_regions.y - (*iter_worstSolu).pos.y)+c3*(center_best_DCP.y - (*iter_worstSolu).pos.y);
        (*iter_worstSolu).pos.z = (*iter_worstSolu).pos.z + c1*((*iter_bestSolu).pos.z-(*iter_worstSolu).pos.z)+c2*(center_all_regions.z - (*iter_worstSolu).pos.z)+c3*(center_best_DCP.z - (*iter_worstSolu).pos.z);
		*/
		(*iter_worstSolu).pos = (*iter_worstSolu).pos + ((*iter_bestSolu).pos-(*iter_worstSolu).pos) * c1 + (center_all_regions - (*iter_worstSolu).pos) * c2 + (center_best_DCP - (*iter_worstSolu).pos) * c3;
        iter_bestSolu++;
        iter_worstSolu++;
    }
	getFitness(worstSolution,s);

	//stop = GetTickCount();  
	//printf("the time to update position: %d ms\n", stop - start);  

}

void relativeOrderUpdate(Individual& worstSolution,SENSOR_NODE s[])
{
	DWORD start, stop;  
	//start = GetTickCount(); 

    int i=0,j=0;
    srand(time(NULL));
    i = rand()%(worstSolution.path.size()-2)+1;
    j = rand()%(worstSolution.path.size()-i-1) + (i+1);
    for(int k=0;k<j-i;k++)
    {
        WAYPOINT temp = worstSolution.path[j];
        worstSolution.path.erase(worstSolution.path.begin()+j);
        worstSolution.path.insert(worstSolution.path.begin()+i+k,temp);
    }
	getFitness(worstSolution,s);

	//stop = GetTickCount();  
	//printf("the time to update  relative order: %d ms\n", stop - start);  

}

Individual updateWosrtSolution(Individual& bestSolution,Individual& worstSolution,map<set<short>,vector<int> >& overlapping_regions,SENSOR_NODE s[])
{
	DWORD start, stop;  
	//start = GetTickCount(); 

    Individual updated_sol = worstSolution;
    Individual final_updated_sol = worstSolution;
    traversalOrderUpdate(bestSolution,updated_sol,s);
	if(isFeasibleSolution(updated_sol) && (updated_sol.fitness < final_updated_sol.fitness))
    {
		final_updated_sol = updated_sol;
		if(updated_sol.fitness > bestSolution.fitness)
		{
			positionUpdate(bestSolution,updated_sol,overlapping_regions,s);
			if(isFeasibleSolution(updated_sol) && (updated_sol.fitness < final_updated_sol.fitness))
			{
				final_updated_sol = updated_sol;
			}
			else
			{
				updated_sol = final_updated_sol;
			}
		}
		relativeOrderUpdate(updated_sol,s);
		if(isFeasibleSolution(updated_sol) && (updated_sol.fitness < final_updated_sol.fitness))
		{
			final_updated_sol = updated_sol;
		}

	}

	//stop = GetTickCount();  
	//printf("the time to update  a worst solution: %d ms\n", stop - start);  

	return final_updated_sol;
}

Individual shuffledFrogLeapingAlgo(map<set<short>,vector<int> >& optimal_overlapping_regions,SENSOR_NODE s[],short h)
{


    vector<Individual> population;
    Individual global_best_sol;
    Individual local_best_sol;
    Individual local_worst_sol;
    vector<vector<Individual> > subPopulations;
    ofstream fout;
    char str[30]="opti_solu_in_each_gene.txt";
	char fit[30]="fitness_in_each_gene.txt";

    population.clear();
    subPopulations.clear();
    init(optimal_overlapping_regions,population,s,h);
    sortIndividuals(population);
    for(int i=0; i<Gene; i++)
    {
		DWORD start, stop;  
		//start = GetTickCount(); 

        global_best_sol = population[0];
        
		/*
        //test the optimal solution in each generation.
        fout.open(str,ios::app);
        vector<WAYPOINT> path = global_best_sol.path;
        double fitness = global_best_sol.fitness;
        vector<WAYPOINT>::iterator iter_path = path.begin();
        POS pos;
        while(iter_path != path.end())
        {
            pos = (*iter_path).pos;
            fout<<pos.x<<" "<<pos.y<<" "<<pos.z<< "\t";
			//cout<<pos.x<<" "<<pos.y<<" "<<pos.z<<"||";
            iter_path++;
        }
        //cout<<fitness<<endl;
        fout<<fitness<<"\n";
        fout.close();
        */

		
		fout.open(fit,ios::app);
		double fit = global_best_sol.fitness;
		fout<<fit<<"\n";
		fout.close();
		
		subPopulations.clear();
        seperateIntoSubPopulation(population,subPopulations);
        for(int j=0;j<Meme;j++)
        {
            for(int k=0;k<Num;k++)
            {
                vector<Individual> subGroup;
                subGroup.clear();
                constructGroup(subPopulations.at(j),subGroup);
                sortIndividuals(subGroup);
                //cout<<subPopulations[j].size()<<" " <<subGroup.size()<<endl;
                local_best_sol = subGroup.at(0);
                local_worst_sol = subGroup.at(GInd-1);
                Individual updated_sol = updateWosrtSolution(local_best_sol,local_worst_sol,optimal_overlapping_regions,s);
                if(updated_sol.fitness >= local_worst_sol.fitness)
                {
                    updated_sol = updateWosrtSolution(global_best_sol,local_worst_sol,optimal_overlapping_regions,s);
                    if(updated_sol.fitness >= local_worst_sol.fitness)
                    {
                        subGroup[GInd-1]=geneOneSolutByBestIndi(global_best_sol,optimal_overlapping_regions,s,h);
                    }
                    else
                    {
                        subGroup[GInd-1] = updated_sol;
                    }
                }
                else
                {
                    subGroup[GInd-1] = updated_sol;
                }
                reBuildSubPopulation(subPopulations[j],subGroup);
                sortIndividuals(subPopulations[j]);
            }
        }
        mergyIntoPopulation(population,subPopulations);
        sortIndividuals(population);
		/*
        char str_population[30];
        itoa(i,str_population,10);
        strcat(str_population,"_gene_solus.txt");
        cout<<str_population<<endl;
        fout.open(str_population);
        vector<Individual>::iterator iter_popution = population.begin();
        vector<WAYPOINT> path;
        double fitness;
        while(iter_popution != population.end())
        {
            path = (*iter_popution).path;
            fitness = (*iter_popution).fitness;
            vector<WAYPOINT>::iterator iter_path = path.begin();
            POS pos;
            while(iter_path != path.end())
            {
                pos = (*iter_path).pos;
                fout<<pos.x<<" "<<pos.y<<" "<<pos.z<< "\t";
                iter_path++;
            }
            //cout<<fitness<<endl;
            fout<<fitness<<"\n";
            iter_popution++;
        }

        fout.close();
		*/
		cout<<i<<endl;

		//stop = GetTickCount();  
		//printf("the time to evolve a generation: %d ms\n", stop - start);  

	}

    return population[0];
}


Individual my_algorithm(SENSOR_NODE sn[])
{
	map<int,set<short> > point_set;
    map<set<short>,vector<int> > overlapping_regions;
    map<set<short>,vector<int> > optimal_overlapping_regions;
    map<set<short>,float> ave_col_times;
	short h = minimal_height;
	Individual bestIndividual;
	bestIndividual.fitness = (double)INT_MAX;

    ofstream fout;
    Individual final_solu;
	SENSOR_NODE s[SN_NUM];
	for(int i=0;i<SN_NUM;i++)
	{
		s[i].id = sn[i].id;
		s[i].pos.x = sn[i].pos.x / CUBE_LEN;
		s[i].pos.y = sn[i].pos.y / CUBE_LEN;
		s[i].pos.z = 0;
	}

	center_all_regions = findCenterAllRegion(s,h);

	for(;h<COMM_RANGE/4;h+=10)
	{
		point_set.clear();
		overlapping_regions.clear();
		optimal_overlapping_regions.clear();
		ave_col_times.clear();

		generate_point_set(point_set,s,h);
		generate_overlapping_region(point_set,overlapping_regions);

		/*
		int filename = 0;
		map<set<short>,vector<int> >::iterator iter_over_reg;
		iter_over_reg = overlapping_regions.begin();
		while(iter_over_reg != overlapping_regions.end())
		{
		char strfilename[10];
		char *lastname = ".txt";
		itoa(filename,strfilename,10);
		strcat(strfilename,lastname);
		fout.open(strfilename);
		cout<<strfilename<<endl;
		vector<int> temp_points = iter_over_reg->second;
		vector<int>::iterator iter_points = temp_points.begin();
		while(iter_points!=temp_points.end())
		{
		POS pos;
		int_to_point(*iter_points,pos);
		fout << pos.x <<" " <<pos.y <<" "<<pos.z<<"\n";
		iter_points++;
		}
		fout.close();
		iter_over_reg++;
		filename++;
		}

		*/

		/*
		map<set<short>,vector<int> >::iterator it;
		it = overlapping_regions.begin();
		cout << overlapping_regions.size() << endl;
		while(it != overlapping_regions.end())
		{
		cout << it->first.size() << " ";
		set<short>::iterator its;
		its = it->first.begin();
		while(its != it->first.end())
		{
		cout << *its << " " ;
		its++;
		}
		cout << it->second.size() << " ";
		cout << endl;
		it++;

		}
		*/

		getAveColTime(overlapping_regions,s,ave_col_times);

		/*

		map<set<short>,float>::iterator ite_ave_time;
		ite_ave_time = ave_col_times.begin();
		cout << ave_col_times.size() << endl;
		int i=0;
		while(i<50 && ite_ave_time != ave_col_times.end())
		{
		cout << ite_ave_time->first.size() << " ";
		set<short>::iterator ite_sen_nodes;
		ite_sen_nodes = ite_ave_time->first.begin();
		while(ite_sen_nodes != ite_ave_time->first.end())
		{
		cout << *ite_sen_nodes << " " ;
		ite_sen_nodes++;
		}
		cout << ite_ave_time->second << " ";
		cout << endl;
		i++;
		ite_ave_time++;

		}
		*/


		findOptiOverlReg(overlapping_regions,ave_col_times,optimal_overlapping_regions);

		center_in_each_region.clear();
		for(map<set<short>,vector<int> >::iterator iter_opt_ovel_reg = optimal_overlapping_regions.begin();iter_opt_ovel_reg!=optimal_overlapping_regions.end();iter_opt_ovel_reg++)
		{
			set<short> temp_region = iter_opt_ovel_reg->first;
			POS temp_center = findBestDCPSet(temp_region,iter_opt_ovel_reg->second,s);
			center_in_each_region.insert(pair<set<short>,POS>(temp_region,temp_center));
		}

		/*
		filename = 0;
		map<set<short>,vector<int> >::iterator iter_opt_over_reg;
		iter_opt_over_reg = optimal_overlapping_regions.begin();
		while(iter_opt_over_reg != optimal_overlapping_regions.end())
		{
		char strfilename[30];
		char *lastname = "_optimal_region.txt";
		itoa(filename,strfilename,10);
		strcat(strfilename,lastname);
		fout.open(strfilename);
		cout<<strfilename<<endl;
		vector<int> temp_points = iter_opt_over_reg->second;
		vector<int>::iterator iter_points = temp_points.begin();
		while(iter_points!=temp_points.end())
		{
		POS pos;
		int_to_point(*iter_points,pos);
		fout << pos.x <<" " <<pos.y <<" "<<pos.z<<"\n";
		iter_points++;
		}
		fout.close();
		iter_opt_over_reg++;
		filename++;
		}
		*/

		/*
		map<set<short>,vector<int> >::iterator it;
		it = optimal_overlapping_regions.begin();
		cout << optimal_overlapping_regions.size() << endl;
		int i=0;
		while(it != optimal_overlapping_regions.end())
		{
		cout << it->first.size() << " ";
		set<short>::iterator its;
		its = it->first.begin();
		while(its != it->first.end())
		{
		cout << *its << " " ;
		its++;
		}
		cout << it->second.size() << " ";
		cout << endl;
		i++;
		it++;

		}
		*/

		final_solu = shuffledFrogLeapingAlgo(optimal_overlapping_regions,s,h);
		///*
        //test the optimal solution in each generation.
       // fout.open(str,ios::app);
        vector<WAYPOINT> path = final_solu.path;
        double fitness = final_solu.fitness;
        vector<WAYPOINT>::iterator iter_path = path.begin();
        POS pos;
        while(iter_path != path.end())
        {
            pos = (*iter_path).pos;
           // fout<<pos.x<<" "<<pos.y<<" "<<pos.z<< "\t";
			cout<<pos.x<<" "<<pos.y<<" "<<pos.z<<"||";
            iter_path++;
        }
        cout<<fitness<<endl;
        //fout<<fitness<<"\n";
       // fout.close();
        //*/

		if(final_solu.fitness < bestIndividual.fitness)
		{
			bestIndividual = final_solu;
		}

	}
    
	return bestIndividual;
}