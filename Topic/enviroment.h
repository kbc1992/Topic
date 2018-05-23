#ifndef ENVIROMENT_H_INCLUDED
#define ENVIROMENT_H_INCLUDED

#include<set>
#include<map>
#include <vector>
using namespace std;

//environmental range
const static short X=1600;
const static short Y=1200;
const static short minimal_height = 10;
const static short COMM_RANGE=160.0;
const static short CUBE_LEN=1;

const static short g_x=X/CUBE_LEN;
const static short g_y=Y/CUBE_LEN;
const static short g_R=COMM_RANGE/CUBE_LEN;

const static short SN_NUM=10;

const static short sensorData = 20; //unit kB

// position of points
typedef struct POS
{
    short x;
    short y;
    short z;

	POS operator+(const POS &pos)
	{
		POS p;
		p.x = x + pos.x;
		p.y = y + pos.y;
		p.z = z;
		return p;
	}

	POS operator-(const POS &pos)
	{
		POS p;
		p.x = x - pos.x;
		p.y = y - pos.y;
		p.z = z;
		return p;
	}

	POS operator*(const double &fac)
	{
		POS p;
		p.x = x * fac;
		p.y = y * fac;
		p.z = z;
		return p;
	}

	POS operator/(const short &fac)
	{
		POS p;
		p.x = x / fac;
		p.y = y / fac;
		p.z = z;
		return p;
	}
}POS;

typedef struct
{
    int id;
    POS pos;
}SENSOR_NODE;

typedef struct WAYPOINT
{
    POS pos;
    set<short> ss;

    WAYPOINT& operator=(const WAYPOINT &waypoint)
    {
        pos = waypoint.pos;
        ss = waypoint.ss;
        return *this;
    }
}WAYPOINT;

//SENSOR_NODE s[SN_NUM];  //sensor node set
//map<int,set<short> > point_set; //the sensor node set of corresponding waypoints

void generate_SNs(SENSOR_NODE s[]);                    //generate the positions of SNs
double dist(POS a,POS b);     // calculate the distance between two nodes
double dist_PSO(POS a,POS b);
float getDataRatePSO(POS waypoint,POS sensor_pos);
void generate_point_set(map<int,set<short> >& point_set,SENSOR_NODE s[],short h);
void generate_overlapping_region(map<int,set<short> >& point_set,map<set<short>,vector<int> >& overlapping_regions);
void int_to_point(int i,POS& p);
int point_to_int(POS p);

#endif // ENVIROMENT_H_INCLUDED
