#include <iostream>
#include <fstream>
#include <vector>
#include "../MVS.h"
using namespace std;
using namespace MVS;

bool load_scene(string file,Scene &scene);
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);
