#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include "enviroment.h"
#include "Sensors_Optimal_Cover.h"
#include "SFLA.h"
#include "Compare_Test_1.h"
#include "Compare_Test_2.h"

using namespace std;


int main()
{

    SENSOR_NODE s[SN_NUM];
	ofstream fout;
	Individual my_algorithm_solu;
	Individual comp_test_1_solu;
	Individual comp_test_2_solu;
	POS pos;
	double fitness;
	double tour_length;
	double collect_time;

	generate_SNs(s);
	fout.open("sensor_nodes_position.txt");
	for(int i=0;i<SN_NUM;i++)
		fout << s[i].pos.x <<" "<< s[i].pos.y <<" "<< s[i].pos.z << "\n";
	fout.close();


	my_algorithm_solu = my_algorithm(s);
	comp_test_1_solu = TSP_GA(s);
	comp_test_2_solu = testAlgorithm2(s);


	fout.open("final_solution.txt");

	
	vector<WAYPOINT> path = my_algorithm_solu.path;
	vector<WAYPOINT>::iterator iter_path = path.begin();
	fitness = my_algorithm_solu.fitness;
	tour_length = my_algorithm_solu.tour_length;
	collect_time = my_algorithm_solu.collect_time;

	while(iter_path != path.end())
	{
		pos = (*iter_path).pos;
		//cout<<pos.x<<" "<<pos.y<<" "<<pos.z<< endl;
		fout<<pos.x*CUBE_LEN<<" "<<pos.y*CUBE_LEN<<" "<<pos.z*CUBE_LEN<<  " ";
		iter_path++;
	}
	//cout<<fitness<<"\n";
	fout<<" "<<tour_length<<" "<<collect_time<<" "<<fitness <<"\n";

	
	vector<WAYPOINT> cp1_path = comp_test_1_solu.path;
	vector<WAYPOINT>::iterator iter_cp1_path = cp1_path.begin();
	fitness = comp_test_1_solu.fitness;
	tour_length = comp_test_1_solu.tour_length;
	collect_time = comp_test_1_solu.collect_time;
	while(iter_cp1_path != cp1_path.end())
	{
		pos = (*iter_cp1_path).pos;
		//cout<<pos.x<<" "<<pos.y<<" "<<pos.z<< endl;
		fout<<pos.x*CUBE_LEN<<" "<<pos.y*CUBE_LEN<<" "<<pos.z<< " ";
		iter_cp1_path++;
	}
	fout<<" "<<tour_length<<" "<<collect_time<<" "<<fitness <<"\n";
	

	vector<WAYPOINT> cp2_path = comp_test_2_solu.path;
	vector<WAYPOINT>::iterator iter_cp2_path = cp2_path.begin();
	fitness = comp_test_2_solu.fitness;
	tour_length = comp_test_2_solu.tour_length;
	collect_time = comp_test_2_solu.collect_time;
	while(iter_cp2_path != cp2_path.end())
	{
		pos = (*iter_cp2_path).pos;
		cout<<pos.x<<" "<<pos.y<<" "<<pos.z<< endl;
		fout<<pos.x <<" "<<pos.y <<" "<<pos.z<< " ";
		iter_cp2_path++;
	}
	cout<<fitness<<endl;
	fout<<" "<<tour_length<<" "<<collect_time<<" "<<fitness <<"\n";

    fout.close();
	
	

    return 0;
}
