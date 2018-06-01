#include "Compare_Test_1.h"

using namespace std;

Individual TSP_GA(SENSOR_NODE s[])
{
	double **adjacentMatrix = new double*[SN_NUM];
	vi path_order;
	double result;
	Individual comp_solut;
	WAYPOINT start;
	POS pos_start = {x_start,y_start,minimal_height};
	start.pos = pos_start;
	comp_solut.path.push_back(start);
	generate_adjacent_matrix(adjacentMatrix,SN_NUM,s);
	path_order.clear();
	GA(adjacentMatrix,SN_NUM,path_order);
	for(vi::iterator iter_path = path_order.begin();iter_path != path_order.end();iter_path++)
	{
		WAYPOINT temp;
		set<short> ss;
		ss.clear();
		ss.insert((short)*iter_path);
		temp.pos = s[*iter_path].pos;
		temp.pos.z = 10 ;
		temp.ss = ss;
		comp_solut.path.push_back(temp);
	}
	getFitness(comp_solut,s);

	//comp_solut.fitness = result;

	delete_adjacent_matrix(adjacentMatrix,SN_NUM);
	delete [] adjacentMatrix;

	return comp_solut;
}