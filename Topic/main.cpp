#include <iostream>
#include <fstream>
#include <string.h>
#include <string>
#include <stdlib.h>
#include "enviroment.h"
#include "Sensors_Optimal_Cover.h"
#include "SFLA.h"
#include "Compare_Test_1.h"
#include "Compare_Test_2.h"

using namespace std;

void outputResults(Individual &indi,ofstream &fout)
{

	POS pos;
	double fitness;
	double tour_length;
	double collect_time;
	
	vector<WAYPOINT> path = indi.path;
	vector<WAYPOINT>::iterator iter_path = path.begin();
	fitness = indi.fitness;
	tour_length = indi.tour_length;
	collect_time = indi.collect_time;

	while (iter_path != path.end())
	{
		pos = (*iter_path).pos;
		//cout<<pos.x<<" "<<pos.y<<" "<<pos.z<< endl;
		fout << pos.x*CUBE_LEN << " " << pos.y*CUBE_LEN << " " << pos.z*CUBE_LEN << " ";
		iter_path++;
	}
	//cout<<fitness<<"\n";
	fout << " " << tour_length << " " << collect_time << " " << fitness << "\n";
}

int main()
{

    SENSOR_NODE s[SN_NUM];
	ofstream fout;
	Individual my_algorithm_solu;
	Individual comp_test_1_solu;
	Individual comp_test_2_solu;


	generate_SNs(s);
	fout.open("sensor_nodes_position.txt");
	for(int i=0;i<SN_NUM;i++)
		fout << s[i].pos.x <<" "<< s[i].pos.y <<" "<< s[i].pos.z << "\n";
	fout.close();

	for (int i = 0; i < 20; i++)
	{
		char entire_name[30] = "\0";
		string  str_entire_name = to_string(i);
		string filename = "final_solution.txt";
		str_entire_name = str_entire_name + filename;
		strcpy(entire_name, str_entire_name.c_str());
		fout.open(entire_name);

		//my_algorithm_solu = my_algorithm(s);
		//outputResults(my_algorithm_solu, fout);

		//comp_test_1_solu = TSP_GA(s);
		//outputResults(comp_test_1_solu, fout);

		comp_test_2_solu = testAlgorithm2(s);
		outputResults(comp_test_2_solu, fout);


		fout.close();

		
		
	}
	
	

    return 0;
}
