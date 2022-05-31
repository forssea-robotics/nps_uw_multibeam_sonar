#include "sonar_calculation_cuda.cuh"

#include <assert.h>
#include <sys/stat.h>

#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <algorithm>
#include <string>
#include <vector>
#include <limits>

#include <opencv2/core.hpp>
#include <complex>
#include <valarray>
#include <sstream>
#include <ostream>
#include <chrono>
#include <string>

typedef std::complex<float> Complex;
typedef std::valarray<Complex> CArray;
typedef std::valarray<CArray> CArray2D;
typedef std::valarray<float> Array;
typedef std::valarray<Array> Array2D;

inline double unnormalized_sinc(double t)
{
  try
  {
    double results = sin(t)/t;
    if (results != results)
      return 1.0;
    else
      return sin(t)/t;
  }catch(int expn)
  {
    return 1.0;
  }
}

void ComputeCorrector(float** &beamCorrector, float &beamCorrectorSum, int nBeams, int width, double hFOV, double hPixelSize)
{
  double fl = static_cast<double>(width) / (2.0 * tan(hFOV/2.0));
  // Beam culling correction precalculation
  for (size_t beam = 0; beam < nBeams; beam ++)
  {
    float beam_azimuthAngle = atan2(static_cast<double>(beam) -
                        0.5 * static_cast<double>(width), fl);
    for (size_t beam_other = 0; beam_other < nBeams; beam_other ++)
    {
      float beam_azimuthAngle_other = atan2(static_cast<double>(beam_other) -
                        0.5 * static_cast<double>(width), fl);
      float azimuthBeamPattern =
        unnormalized_sinc(M_PI * 0.884 / hPixelSize
        * sin(beam_azimuthAngle-beam_azimuthAngle_other));
      beamCorrector[beam][beam_other] = abs(azimuthBeamPattern);
      beamCorrectorSum += pow(azimuthBeamPattern, 2);
    }
  }
  beamCorrectorSum = sqrt(beamCorrectorSum);
}

int main (int argc, char *argv[]) 
{ 
  int width = 256;
  int height = 102;
  cv::Mat depth_image = cv::Mat(width, height, CV_32FC1, cv::Scalar(0.));
  cv::Mat normal_image = cv::Mat(width, height, CV_32FC1, cv::Scalar(0.));
  cv::Mat rand_image = cv::Mat(width, height, CV_32FC1, cv::Scalar(0.));;
  double vFOV = 0.45221;
  double hFOV = 1.0472;
  double vPixelSize = vFOV / (height-1);
  double hPixelSize = hFOV / (width-1);

  int nBeams = width;
  int nRays = height;
  int ray_nElevationRays = height;
  int ray_nAzimuthRays = 1;
  float* elevation_angles = new float[nRays];

  double verticalFOV = 12; 
  double sonarFreq = 2100e3;
  double bandwidth = 265.5e3;
  double soundSpeed = 1500;
  double maxDistance = 1.2;
  double sourceLevel = 150;
  float raySkips = 1;
  float mu = 1e-3;
  double absorption = 0.0354;  // [dB/m]
  double attenuation = absorption*log(10)/20.0;

  const float max_T = maxDistance*2.0/soundSpeed;
  float delta_f = 1.0/max_T;
  const float delta_t = 1.0/bandwidth;
  int nFreq = ceil(bandwidth/delta_f);
  delta_f = bandwidth/nFreq;
  const int nTime = nFreq;
  float* rangeVector = new float[nTime];
  for (int i = 0; i < nTime; i++)
  {
    rangeVector[i] = delta_t*i*soundSpeed/2.0;
  }

  float* window = new float[nFreq];
  float windowSum = 0;
  for (size_t f = 0; f < nFreq; f++)
  {
    window[f] = 0.54 - 0.46 * cos(2.0*M_PI*(f+1)/nFreq);
    windowSum += pow(window[f], 2.0);
  }
  for (size_t f = 0; f < nFreq; f++)
    window[f] = window[f]/sqrt(windowSum);

  // Sonar corrector preallocation
  float** beamCorrector = new float*[nBeams];
  for (int i = 0; i < nBeams; i++)
      beamCorrector[i] = new float[nBeams];
  float beamCorrectorSum = 0.0;
  bool debugFlag = true;

  if (beamCorrectorSum == 0)
    ComputeCorrector(beamCorrector, beamCorrectorSum, nBeams, width, hFOV, hPixelSize);

  cv::Mat reflectivityImage = cv::Mat(width, height, CV_32FC1, cv::Scalar(mu));

  while(1){
    // For calc time measure
    auto start = std::chrono::high_resolution_clock::now();
    // ------------------------------------------------//
    // --------      Sonar calculations       -------- //
    // ------------------------------------------------//
    CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper(
                    depth_image,   // cv::Mat& depth_image
                    normal_image,  // cv::Mat& normal_image
                    rand_image,    // cv::Mat& rand_image
                    hPixelSize,    // hPixelSize
                    vPixelSize,    // vPixelSize
                    hFOV,          // hFOV
                    vFOV,          // VFOV
                    hPixelSize,    // _beam_azimuthAngleWidth
                    verticalFOV/180*M_PI,  // _beam_elevationAngleWidth
                    hPixelSize,    // _ray_azimuthAngleWidth
                    elevation_angles, // _ray_elevationAngles
                    vPixelSize*(raySkips+1),  // _ray_elevationAngleWidth
                    soundSpeed,    // _soundSpeed
                    maxDistance,   // _maxDistance
                    sourceLevel,   // _sourceLevel
                    nBeams,        // _nBeams
                    nRays,         // _nRays
                    raySkips,      // _raySkips
                    sonarFreq,     // _sonarFreq
                    bandwidth,     // _bandwidth
                    nFreq,         // _nFreq
                    reflectivityImage,  // reflectivity_image
                    attenuation,   // _attenuation
                    window,        // _window
                    beamCorrector,      // _beamCorrector
                    beamCorrectorSum,   // _beamCorrectorSum
                    debugFlag);
    
    // For calc time measure
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<
                    std::chrono::microseconds>(stop - start);
    if (debugFlag)
    {
      std::cout <<"GPU Sonar Frame Calc Time " <<
                      duration.count()/10000 << "/100 [s]\n"<<std::endl;
    }
  }

  return EXIT_SUCCESS;
} 
