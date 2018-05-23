#include "enviroment.h"
#include "Sensors_Optimal_Cover.h"
#include "SFLA.h"
#include  <vector>

using namespace std;

static const short he = 10;
static const short particle_num = 100;
static const short generations = 15000;
static const double c1 = 2.0;
static const double c2 = 2.0;
static const double wei = 0.8;
static const double attenuation_factor = 0.99999;

typedef struct Particle
{
	vector<POS> position;
	vector<POS> v;
	vector<POS> pbest;
	double fitness;
	double pbest_fitness;

	/*
	Particle():fitness(0){position.clear();};
	Particle(vector<POS> vec,double fit)
	{
		this->position = vec;
		this->fitness = fit;
	}
	~Particle(){};
	Particle& operator=(const Particle &par)
	{
		position = par.position;
		fitness = par.fitness;
		return *this;
	}
	*/
}Particle;

static vector<Particle> swarm;



//void generate_SNs_PSO(SENSOR_NODE s[]);
//double dist_PSO(POS a,POS b);
double setFitness(Individual& solution,SENSOR_NODE s[]);
void setPSOFitness(Particle &particle);
bool isFeasiblePath(vector<WAYPOINT> &path,SENSOR_NODE s[]);
void generateParticles(SENSOR_NODE s[]);
bool lessSortParticle(Particle a,Particle b);
Particle sortParticles(vector<Particle>& particles);
void reverseOrder(Particle &particle,double reverse_probability);
void mergeRPs(Particle &particle);
void updateParticles(Particle &particle,vector<POS> &gbest);
void simulatedAnnealing(Particle &particle,vector<POS> &oldPath,double old_fitness,double temperature,SENSOR_NODE s[]);
Particle VD_PSO(SENSOR_NODE s[]);

Individual testAlgorithm2(SENSOR_NODE s[]);