#include "localization/ParticleFilter.h"
#include "localization/Util.h"

#include "tf/tf.h"

using namespace std;

ParticleFilter::ParticleFilter(int numberOfParticles) {
	this->numberOfParticles = numberOfParticles;

	// initialize particles
	for (int i = 0; i < numberOfParticles; i++) {
		this->particleSet.push_back(new Particle());
	}

	// this variable holds the estimated robot pose
	this->bestHypothesis = new Particle();

	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
	this->laserSkip = 5;

	// distance map used for computing the likelihood field
	this->distMap = NULL;
}

ParticleFilter::~ParticleFilter() {
	// delete particles
	for (int i = 0; i < numberOfParticles; i++) {
		Particle* p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles() {
	return this->numberOfParticles;
}

std::vector<Particle*>* ParticleFilter::getParticleSet() {
	return &(this->particleSet);
}

void ParticleFilter::initParticlesUniform() {
    //get map properties
    int mapWidth, mapHeight;
    double mapResolution;
    this->getLikelihoodField(mapWidth, mapHeight,mapResolution);

	// TODO: here comes your code
	for(int i = 0; i < numberOfParticles; i++){
		Particle* particle = particleSet[i];
		particle->x = Util::uniformRandom(0, mapWidth*mapResolution);
		particle->y = Util::uniformRandom(0, mapHeight*mapResolution);
		particle->theta = Util::uniformRandom(-M_PI, M_PI);
	}
}

void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y,
		double mean_theta, double std_xx, double std_yy, double std_tt) {
	// TODO: here comes your code
	for(int i = 0; i < numberOfParticles; i++){
		Particle* particle = particleSet[i];
		particle->x = Util::gaussianRandom(mean_x, std_xx);
		particle->y = Util::gaussianRandom(mean_y, std_yy);
		double theta = Util::gaussianRandom(mean_theta, std_tt);
		particle->theta = Util::normalizeTheta(theta);
	}
}

/**
 *  Initializes the likelihood field as our sensor model.
 */
void ParticleFilter::setMeasurementModelLikelihoodField(
		const nav_msgs::OccupancyGrid& map, double zRand, double sigmaHit) {
	ROS_INFO("Creating likelihood field for laser range finder...");

	// create the likelihood field - with the same discretization as the occupancy grid map
	this->likelihoodField = new double[map.info.height * map.info.width];
	this->likelihoodFieldWidth = map.info.width;
	this->likelihoodFieldHeight = map.info.height;
	this->likelihoodFieldResolution = map.info.resolution;

    // calculates the distance map and stores it in member variable 'distMap'
	// for every map position it contains the distance to the nearest occupied cell.
	calculateDistanceMap(map);

    // Here you have to create your likelihood field
	// HINT0: sigmaHit is given in meters. You have to take into account the resolution of the likelihood field to apply it.
	// HINT1: You will need the distance map computed 3 lines above
	// HINT2: You can visualize it in the map_view when clicking on "show likelihood field" and "publish all".
	// HINT3: Storing probabilities in each cell between 0.0 and 1.0 might lead to round-off errors, therefore it is
	// good practice to convert the probabilities into log-space, i.e. storing log(p(x,y)) in each cell. As a further
	// advantage you can simply add the log-values in your sensor model, when you weigh each particle according the
	// scan, instead of multiplying the probabilities, because: log(a*b) = log(a)+log(b).

	// TODO: here comes your code
	double sigma = sigmaHit/likelihoodFieldResolution;
	for(int i = 0; i < likelihoodFieldWidth*likelihoodFieldHeight; i++){
		likelihoodField[i] = log((1-zRand)*Util::gaussian(distMap[i], sigma, 0) + zRand);
	}
	ROS_INFO("...DONE creating likelihood field!");
}

void ParticleFilter::calculateDistanceMap(const nav_msgs::OccupancyGrid& map) {
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++) {
		for (int y = 0; y < likelihoodFieldHeight; y++) {
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
	for (int x = 0; x < map.info.width; x++) {
		for (int y = 0; y < map.info.height; y++) {
			if (map.data[x + y * map.info.width] >= occupiedCellProbability) {
				bool border = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (!border && x + i >= 0 && y + j >= 0 && x + i
								< likelihoodFieldWidth && y + j
								< likelihoodFieldHeight && (i != 0 || j != 0)) {
							if (map.data[x + i + (y + j) * likelihoodFieldWidth]
									< occupiedCellProbability && map.data[x + i
									+ (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth]
									= 0.0;
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double* ParticleFilter::getLikelihoodField(int& width, int& height,
		double& resolution) {
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	likelihoodFieldRangeFinderModel(laserScan);
}

/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
		const sensor_msgs::LaserScanConstPtr & laserScan) {

	// TODO: here comes your code
	for(int particleIndex = 0; particleIndex < numberOfParticles; particleIndex++){
		Particle* particle = particleSet[particleIndex];
		double weight = 0;
		bool outOfMap = false;
		for(int laserIndex = 0; laserIndex < laserScan->ranges.size(); laserIndex += laserSkip){
			double range = laserScan->ranges[laserIndex];
			if(isnan(range) || range > laserScan->range_max || range < laserScan->range_min){
				continue;
			}
			double laserAngle = laserScan->angle_min + laserIndex*laserScan->angle_increment;
			double mapAngle = laserAngle + particle->theta;
			double mapRange = range / likelihoodFieldResolution;


			int particleXMap = particle->x / likelihoodFieldResolution;
			int particleYMap = particle->y / likelihoodFieldResolution;

			int endX = (int)(cos(mapAngle)*mapRange) + particleXMap;
			int endY = (int)(sin(mapAngle)*mapRange) + particleYMap;
			if(endX < 0 || endX >= likelihoodFieldWidth || endY < 0 || endY >= likelihoodFieldHeight){
                weight = log(0.000000001);
				break;
			}
			weight += likelihoodField[computeMapIndex(likelihoodFieldWidth, likelihoodFieldHeight, endX, endY)];
		}
        particle->weight = exp(weight);
	}

	double norm = 0;
	for(int particleIndex = 0; particleIndex < numberOfParticles; particleIndex++){
		norm += particleSet[particleIndex]->weight;
	}
	for(int particleIndex = 0; particleIndex < numberOfParticles; particleIndex++){
		particleSet[particleIndex]->weight /= norm;
	}
}

void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2,
		double alpha3, double alpha4) {
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;
}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	// TODO: here comes your code
	double delta_trans_start = sqrt((newX - oldX)*(newX - oldX) + (newY - oldY)*(newY - oldY));
	double delta_rot1_start = Util::diffAngle(oldTheta, atan2(newY - oldY, newX - oldX));
	double delta = Util::diffAngle(oldTheta, newTheta);
	double delta_rot2_start = Util::diffAngle(delta_rot1_start, delta);

	for(int i = 0; i < numberOfParticles; i++){
		double delta_rot1 = delta_rot1_start + Util::gaussianRandom(0, odomAlpha1*abs(delta_rot1_start) + odomAlpha2*delta_trans_start);
		double delta_trans = delta_trans_start + Util::gaussianRandom(0, odomAlpha3*delta_trans_start + odomAlpha4*abs(Util::normalizeTheta(delta_rot1_start + delta_rot2_start)));
		double delta_rot2 = delta_rot2_start + Util::gaussianRandom(0, odomAlpha1*abs(delta_rot2_start) + odomAlpha2*delta_trans_start);

		Particle* particle = particleSet[i];
		double angle = Util::normalizeTheta(particle->theta + delta_rot1);
		particle->x += delta_trans * cos(angle);
		particle->y += delta_trans * sin(angle);
		particle->theta = Util::normalizeTheta(particle->theta + delta_rot1 + delta_rot2);
	}
}

/**
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample() {
	// TODO: here comes your code
	double* cdf = new double[numberOfParticles];
	Particle* newParticles = new Particle[numberOfParticles];
	cdf[0] = particleSet[0]->weight;
	for(int i = 1; i < numberOfParticles; i++){
		cdf[i] = cdf[i-1] + particleSet[i]->weight;
	}


	double u = Util::uniformRandom(0, (double)1/(double)numberOfParticles);
	int i = 0;
	for(int j = 0; j < numberOfParticles; j++){
		while(u > cdf[i]){
			i++;
		}
		newParticles[j].x = particleSet[i]->x;
		newParticles[j].y = particleSet[i]->y;
		newParticles[j].theta = particleSet[i]->theta;
		newParticles[j].weight = particleSet[i]->weight;
		u += (double)1/(double)numberOfParticles;
	}

	for(int i = 0; i < numberOfParticles; i++){
		particleSet[i]->x = newParticles[i].x;
		particleSet[i]->y = newParticles[i].y;
		particleSet[i]->theta = newParticles[i].theta;
		particleSet[i]->weight = newParticles[i].weight;
	}

	Particle* best = particleSet[0];
	for(int i = 0; i < numberOfParticles; i++){
		if(particleSet[i]->weight > best->weight){
			best = particleSet[i];
		}
	}
	bestHypothesis = best;

	delete[] cdf;
	delete[] newParticles;
}

Particle* ParticleFilter::getBestHypothesis() {
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}

