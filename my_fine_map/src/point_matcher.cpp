//
// Created by doug on 2021-03-31.
//Calculates the transform between 3 sets of points
//Very inefficient, n! scaling, only give it a few points
//
/*
 * Formula:
 * Find points in new scan with the closest intensity value to the previous one
 * Reject any points with multiple matches
 */

#include "point_matcher.h"
#include <chrono>

/*struct PointPair {
    //location of previous and current withing the vectors
    int previous;
    int current;
    double intensityDifference;

    //ratio of the difference between the matched points and the difference if a later point matched
    double postPreviousRatio;
    double postCurrentRatio;

    //ratio of intensity differece to the higher of previous or current ration
    //double uniqueness = std::max(intensityDifference/postPreviousRatio, intensityDifference/postCurrentRatio);
};
 */

/*bool compareXYZI(pcl::PointXYZI a, pcl::PointXYZI b) {
    return a.intensity > b.intensity;
}*/
bool compareX(pcl::PointXYZ a, pcl::PointXYZ b) {
    return a.x > b.x;
}
bool compareY(pcl::PointXYZ a, pcl::PointXYZ b) {
    return a.y > b.y;
}
bool compareZ(pcl::PointXYZ a, pcl::PointXYZ b) {
    return a.z > b.z;
}
bool checkCorresponding(pcl::PointXYZ point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, double tolerance);
Eigen::Matrix4f find3PointTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest, int s1, int s2, int s3, int d1, int d2, int d3);
double calcDistance(pcl::PointXYZ a, pcl::PointXYZ b);
Eigen::Matrix4f findMaxCoreTransGiven1(pcl::PointCloud<pcl::PointXYZ>::Ptr existingCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud, int s1, int *bestMatchCount);

/*
 * existingCloud is the cloud that contains existing points. To minimize compute time preselect likely points
 * newCloud is the latest scan results. To minimize time prefilter existing points
 * the matrix returned is the matrix of the most accurate transform
 */
int simulatedLoad() {
    long start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    while(start+10000>std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    return 10;
}
//finds the transform using maximum correspondence
Eigen::Matrix4f maximumCorrespondenceTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr existingCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud) {
    //sorts the new point cloud by X, necessary for checkCorresponding to run, to avoid trying to sort newCloud in multiple threads
    std::sort (newCloud->begin(), newCloud->end(), compareX);

    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f bestMatch;
    int bestMatchCount = 0;
    Eigen::Matrix4f sampleMatch;

    int given1BestCounts[existingCloud->size()-2];

    /*
    std::cout << "Before simulated load" << std::endl;
    std::future<int> test;
    test = std::async(std::launch::async, simulatedLoad);
    test.wait();
    std::cout << "After simulated load" << std::endl;
     */

    //stores the tasks used to figure out each good match
    std::future<Eigen::Matrix4f> findGiven1Tasks[existingCloud->size()-2];

    long start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    for(int a = 0; a < existingCloud->size()-2; a++) {
        //sequential way of doing this incase something goes wrong
        //findMaxCoreTransGiven1(existingCloud, newCloud, a, &given1BestCounts[a]);

        //findGiven1Tasks.push_back(std::async (std::launch::async, findMaxCoreTransGiven1, existingCloud, newCloud, a, &given1BestCounts[a]));
        findGiven1Tasks[a] = std::async(std::launch::async, findMaxCoreTransGiven1, existingCloud, newCloud, a, &given1BestCounts[a]);
    }

    std::cout << "Number of tasks: " << existingCloud->size()-2 << std::endl;
    for(int n = 0; n< existingCloud->size()-2; n++) {
        findGiven1Tasks[n].wait();
        Eigen::Matrix4f sampleMatch = findGiven1Tasks[n].get();
        if(given1BestCounts[n]>bestMatchCount) {
            bestMatchCount=given1BestCounts[n];
            bestMatch = sampleMatch;
        }
    }

    long end = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    ROS_INFO("Processing one frame, taking %li ms", end-start);
    std::cout << "Best match: " << std::endl;
    std::cout << bestMatch << std::endl;
    std::cout << "With " << bestMatchCount << " matches" << std::endl;
    return bestMatch;
}

/*finds the maximum correspondence transform, given one
 * s1 the provided point in the existing cloud
 * used for multithreading, if this is going to be used on a CPU with more cores than points sampled, continue the multithreading pattern one layer deeper
 * given1BestMatchCount is a reference to where the best match count for this point should be stored
 */
Eigen::Matrix4f findMaxCoreTransGiven1(pcl::PointCloud<pcl::PointXYZ>::Ptr existingCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud, int s1, int* given1BestMatchCount) {
    //resets best match count, not usually need but automatically making all new arrays 0 is technically not in the spec, even though it basically always happens
    *given1BestMatchCount = 0;

    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f bestMatch;
    Eigen::Matrix4f sampleMatch;

    for(int b = s1+1; b <existingCloud->size()-1; b++) {
        for(int c = b+1; c<existingCloud->size(); c++) {
            for(int d = 0; d<newCloud->size()-2; d++) {
                for(int e = d+1; e<newCloud->size()-1; e++) {
                    for(int f = e+1; f<newCloud->size(); f++) {
                        /*finds the distances between every possible pair of points in the set of 3 before and after
                         * If the distances don't match up than the 2 sets of 3 points cannot be a transform of each other
                         */
                        double oldDistance[3];
                        oldDistance[0]=calcDistance(existingCloud->at(s1), existingCloud->at(b));
                        oldDistance[1]=calcDistance(existingCloud->at(s1), existingCloud->at(c));
                        oldDistance[2]=calcDistance(existingCloud->at(b), existingCloud->at(c));
                        std::sort(oldDistance, oldDistance+3);

                        double newDistance[3];
                        newDistance[0]=calcDistance(existingCloud->at(d), existingCloud->at(e));
                        newDistance[1]=calcDistance(existingCloud->at(d), existingCloud->at(f));
                        newDistance[2]=calcDistance(existingCloud->at(e), existingCloud->at(f));
                        std::sort(newDistance, newDistance+3);

                        for(int n = 0; n<3; n++) {
                            if(std::abs(oldDistance[n] - newDistance[n] > CORRESPONDENCE_TOLERANCE)) continue;
                        }

                        sampleMatch = find3PointTransform(existingCloud, newCloud, s1, b, c, d, e, f);

                        pcl::PointCloud<pcl::PointXYZ>::Ptr sampleTransformed;//point cloud resulting from the transform whose accuracy is being tested
                        sampleTransformed = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

                        pcl::transformPointCloud(*existingCloud, *sampleTransformed, sampleMatch);

                        int sampleMatchCount = 0;
                        for(int g = 0; g< existingCloud->size(); g++) {
                            if(checkCorresponding(existingCloud->at(g), newCloud, CORRESPONDENCE_TOLERANCE)) sampleMatchCount++;
                        }

                        if(sampleMatchCount > *given1BestMatchCount) {
                            *given1BestMatchCount = sampleMatchCount;
                            bestMatch = sampleMatch;
                        }
                    }
                }
            }
        }
    }
    return bestMatch;
}

//figures out the transform required to go from source to dest, given s1, s2, and s3 in the source, correspond to d1, d2, and d3 in the destination
Eigen::Matrix4f find3PointTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest, int s1, int s2, int s3, int d1, int d2, int d3) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> sampleICP;

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceSample;
    sourceSample = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr destSample;
    destSample = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    //selects the sampled source points
    sourceSample->clear();
    sourceSample->push_back(source->at(s1));
    sourceSample->push_back(source->at(s2));
    sourceSample->push_back(source->at(s3));
    //selected the sampled destination points
    destSample->clear();
    destSample->push_back(dest->at(d1));
    destSample->push_back(dest->at(d2));
    destSample->push_back(dest->at(d3));

    //sets the inputs for the icp, source to the thing being transformed from, target is the thing being transformed
    sampleICP.setInputSource (sourceSample);
    sampleICP.setInputTarget (destSample);

    // Set the max correspondence distance to (0.05m was used in the tutorial
    sampleICP.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    sampleICP.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    sampleICP.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    sampleICP.setEuclideanFitnessEpsilon (1);

    // Perform the alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr guessTransformFrame;
    guessTransformFrame = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();;

    //calculates the ICP match
    sampleICP.align(*guessTransformFrame);

    return sampleICP.getFinalTransformation();

}

double calcDistance(pcl::PointXYZ a, pcl::PointXYZ b) {
    return sqrt(std::pow(a.x-b.x, 2) + std::pow(a.y-b.y, 2) + std::pow(a.z-b.z, 2));
}

//returns true if there is a corresponding point in the given pointcloud. pointCloud must be sorted by x in descending order first
bool checkCorresponding(pcl::PointXYZ point, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, double tolerance) {

    //the sorts are to prep for binary sort, which can speed up the process, however it is not implemented.
    pcl::PointCloud<pcl::PointXYZ>::Ptr xMatchList=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    //brackets for the possible location of the lowest point within tolerance
    int lowerUpperBracket = pointCloud->size();
    int lowerLowerBracket = -1;
    //brackets for the possible location of the highest point within tolerance, because the goes high to, LowerBracket will be larger than UpperBracket
    int upperUpperBracket = pointCloud->size();
    int upperLowerBracket = -1;

    //stores the target that the bracketing method is trying to hit
    double target = point.x-tolerance;

    //finds the lower end of the points in the x range where lowerLowerBracket is the index of the lowest in range point
    while(lowerLowerBracket!=lowerUpperBracket-1) {
        int middleValue = (lowerUpperBracket+lowerLowerBracket)/2;
        if(pointCloud->at(middleValue).x > target) lowerLowerBracket = middleValue;
        else lowerUpperBracket = middleValue;
    }

    //finds the upper end of the points in the x ranger where upperLowerBracket is the index of the highest in range point
    target = point.x+tolerance;
    while(upperLowerBracket!=upperUpperBracket-1) {
        int middleValue = (upperUpperBracket+upperLowerBracket)/2;
        if(pointCloud->at(middleValue).x > target) upperLowerBracket = middleValue;
        else upperUpperBracket = middleValue;
    }

    //returns false if no items passed the previous test
    if(upperUpperBracket==lowerUpperBracket) return false;

    for(int n = upperUpperBracket; n < lowerUpperBracket; n++) {
        xMatchList->push_back(pcl::PointXYZ(pointCloud->at(n)));
    }


    //begins process of checking for y matches
    std::sort (xMatchList->begin(), xMatchList->end(), compareY);
    pcl::PointCloud<pcl::PointXYZ>::Ptr yMatchList=pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    //brackets for the possible location of the lowest point within tolerance
    lowerUpperBracket = xMatchList->size();
    lowerLowerBracket = -1;
    //brackets for the possible location of the highest point within tolerance, because the goes high to, LowerBracket will be larger than UpperBracket
    upperUpperBracket = xMatchList->size();
    upperLowerBracket = -1;

    //stores the target that the bracketing method is trying to hit
    target = point.y-tolerance;

    //finds the lower end of the points in the x range where lowerLowerBracket is the index of the lowest in range point
    while(lowerLowerBracket!=lowerUpperBracket-1) {
        int middleValue = (lowerUpperBracket+lowerLowerBracket)/2;
        if(xMatchList->at(middleValue).y > target) lowerLowerBracket = middleValue;
        else lowerUpperBracket = middleValue;
    }

    //finds the upper end of the points in the x ranger where upperLowerBracket is the index of the highest in range point
    target = point.y+tolerance;
    while(upperLowerBracket!=upperUpperBracket-1) {
        int middleValue = (upperUpperBracket+upperLowerBracket)/2;
        if(xMatchList->at(middleValue).y > target) upperLowerBracket = middleValue;
        else upperUpperBracket = middleValue;
    }

    //returns false if no items passed the previous test
    if(upperUpperBracket==lowerUpperBracket) return false;

    for(int n = upperUpperBracket; n < lowerUpperBracket; n++) {
        yMatchList->push_back(pcl::PointXYZ(xMatchList->at(n)));
    }


    //begins process of checking for z matches
    //brackets for the possible location of the lowest point within tolerance
    lowerUpperBracket = yMatchList->size();
    lowerLowerBracket = -1;
    //brackets for the possible location of the highest point within tolerance, because the goes high to, LowerBracket will be larger than UpperBracket
    upperUpperBracket = yMatchList->size();
    upperLowerBracket = -1;

    //stores the target that the bracketing method is trying to hit
    target = point.z-tolerance;

    //finds the lower end of the points in the x range where lowerLowerBracket is the index of the lowest in range point
    while(lowerLowerBracket!=lowerUpperBracket-1) {
        int middleValue = (lowerUpperBracket+lowerLowerBracket)/2;
        if(yMatchList->at(middleValue).z > target) lowerLowerBracket = middleValue;
        else lowerUpperBracket = middleValue;
    }

    //finds the upper end of the points in the x ranger where upperLowerBracket is the index of the highest in range point
    target = point.z+tolerance;
    while(upperLowerBracket!=upperUpperBracket-1) {
        int middleValue = (upperUpperBracket+upperLowerBracket)/2;
        if(yMatchList->at(middleValue).z > target) upperLowerBracket = middleValue;
        else upperUpperBracket = middleValue;
    }

    //returns true if there are elements in the yMatch list, that had compatible z values
    return upperUpperBracket < lowerUpperBracket;
}
