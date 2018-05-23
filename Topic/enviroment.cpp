#include<cstdlib>
#include<cmath>
#include <ctime>
#include <iostream>
#include "enviroment.h"

double dist(POS a,POS b)
{
    int sum = (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
    return sqrt(sum*CUBE_LEN*CUBE_LEN*1.0);
}

double dist_PSO(POS a,POS b)
{
	double sum = (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
	return sqrt(sum);
}

float getDataRatePSO(POS waypoint,POS sensor_pos)
{
	double len = dist_PSO(waypoint,sensor_pos);
	if(len<0)
		return 0.0;
	else if(len<20)
		return 250.0;
	else if(len<50)
		return 19.2;
	else if(len<120)
		return 9.6;
	else if(len<200)
		return 4.8;
	else
		return 0.0;
}

void generate_SNs(SENSOR_NODE s[])
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
				if(dist_PSO(s[i].pos,s[k].pos)<COMM_RANGE)
				{
					flag = true;
					break;
				}
			}
		}while(flag);
	}
}

/*
void generate_SNs(SENSOR_NODE s[])
{

    int i,k;
    bool flag;
    //srand(time(NULL));
    for(i=0;i<SN_NUM;i++)
    {
        s[i].id = i;
        do{
            s[i].pos.x = rand()%(g_x - 2*g_R)+g_R;
            s[i].pos.y = rand()%(g_y - 2*g_R)+g_R;
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

void int_to_point(int i,POS& p)
{
    int y,z;
    y = g_y;
    z = g_R;
    p.x = i/(y*z);
    p.y = (i%(y*z))/z;
    p.z = (i%(y*z))%z;
}

int point_to_int(POS p)
{
    int key = p.x*g_y*g_R + p.y*g_R + p.z;
    return key;
}

void generate_point_set(map<int,set<short> >& point_set,SENSOR_NODE s[],short h)
{
    int key;
    set<short> ss;
    POS p;
    for(short n=0;n<SN_NUM;n++)
    {
        int x_temp = s[n].pos.x;
        int y_temp = s[n].pos.y;
		//int z_temp = minimal_height/CUBE_LEN;
        for(int i=x_temp - g_R;i>0&&i<g_x&&i<x_temp + g_R;i++)
        {
            for(int j=y_temp - g_R;j>0&&j<g_y&&j<y_temp + g_R;j++)
            {
				//for(int k= z_temp;k<g_R;k++)
				//{
                    p.x = i;
                    p.y = j;
                    p.z = h;
                    ss.clear();
                    key = point_to_int(p);
                    if(dist(s[n].pos,p)<=COMM_RANGE)
                    {
                        ss.insert(n);
                        pair<map<int,set<short> >::iterator,bool> ret = point_set.insert(pair<int ,set<short> >(key,ss));
                        if(!ret.second)
                        {
                            ret.first->second.insert(n);
                        }
                    }
                //}
            }
        }
    }

}

void generate_overlapping_region(map<int,set<short> >& point_set,
                                 map<set<short>,vector<int> >& overlapping_regions)
{
    map<int,set<short> >::iterator ite_point_set;
    ite_point_set = point_set.begin();
    vector<int> points;
    while(ite_point_set!=point_set.end())
    {
        points.clear();
        points.push_back(ite_point_set->first);
        pair<map<set<short>,vector<int> >::iterator,bool> ret = overlapping_regions.insert(pair<set<short>,vector<int> >(ite_point_set->second,points));
        if(!ret.second)
        {
            ret.first->second.push_back(ite_point_set->first);
        }
        ite_point_set++;
    }
    /*
    set<short> union_ove_reg;
    for(map<set<short>,vector<int> >::iterator ite=overlapping_regions.begin();ite!=overlapping_regions.end();ite++)
    {
        set<short> temp = ite->first;
        for(set<short>::iterator ite2 = temp.begin();ite2!=temp.end();ite2++)
        {
            if(union_ove_reg.find(*ite2) == union_ove_reg.end())
            {
                union_ove_reg.insert(*ite2);
            }
        }
    }

    for(int i=0; i<SN_NUM; i++)
    {
        if(union_ove_reg.find(i)== union_ove_reg.end())
        {
            ite_point_set = point_set.begin();
            set<short> single_region;
            vector<int> points;
            points.clear();
            single_region.clear();
            single_region.insert(i);
            while(ite_point_set!=point_set.end())
            {
                //if(ite_point_set->second.size()==1&&ite_point_set->second.find(i))
                if(ite_point_set->second == single_region)
                {
                    points.push_back(ite_point_set->first);
                }

                ite_point_set++;
            }
            overlapping_regions.insert(pair<set<short>,vector<int> >(single_region,points));
        }
    }
    */
}
