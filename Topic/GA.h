#ifndef GA_H
#define GA_H

#include <vector>
#include "enviroment.h"
#include "Sensors_Optimal_Cover.h"
#include "SFLA.h"

using namespace std;

#define REP(i,n) for (int(i)=0; (i)<(n); (i)++)
#define FOR(i,k,n) for (int(i)=(k); (i)<=(n); (i)++)
#define INF 1e9


const int population_size = 6500;
const int new_population_size = 3500;
const int steps = 1500;

typedef vector<int> vi;
typedef pair<vi,double> pvi;

static vector<pvi> population;
static vector<pvi> new_population;
static vi tmp, tmp2, tmp3;
static vi bestpopulation;
static double bestresult;

void _print(vector<int> v);
bool cmp(const pvi &a, const pvi &b);
void generate_adjacent_matrix(double **adjacencyMatrix,int n,SENSOR_NODE s[]);
void generate_adjacent_matrix(double **adjacencyMatrix, int n, SENSOR_NODE s[], short h);
void delete_adjacent_matrix(double **adjacencyMatrix,int n);
double calculate(vector<int> &v, const int &n, double **adjacencyMatrix);
void generate(const int &n, double **adjacencyMatrix);
void inversion_mutation(vi &v, const int &n);
void swap_mutation(vi &v, const int &n) ;
void displacement_mutation(vi &v, const int &n) ;
vi OX(vi &p1, vi &p2, const int &n);
void selection(const int &n, double **adjacencyMatrix);
void mutation(vi &v, const int &n);
void judge(const int &n, double **adjacencyMatrix);
void reproduction(const int &n, double **adjacencyMatrix);

double GA(double **adjacencyMatrix, const int &n, vi &result);

#endif
