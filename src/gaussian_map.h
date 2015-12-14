#ifndef _EIN_SCENE_H_
#define _EIN_SCENE_H_

#include "eePose.h"
#include "ein_util.h"

class MachineState;
class Word;

typedef struct _GaussianMapChannel {
  double counts;
  double squaredcounts;
  double mu;
  double sigmasquared;
  double samples;
  void zero();
  void recalculateMusAndSigmas();

  void multS(double scalar); 
  void addC(_GaussianMapChannel * channel); 
} GaussianMapChannel;

typedef struct _GaussianMapCell {
  // mus and sigmas should be updated whenever anything else is modified
  GaussianMapChannel red;
  GaussianMapChannel green;
  GaussianMapChannel blue;
  GaussianMapChannel z;

  void zero();
  void multS(double scalar); 
  void addC(_GaussianMapCell * cell); 

  void writeToFileStorage(FileStorage& fsvO) const;
  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);

  void newObservation(Vec3b obs);
  
  double innerProduct(_GaussianMapCell * other, double * rterm, double * gterm, double * bterm);
  double pointDiscrepancy(_GaussianMapCell * other, double * rterm, double * gterm, double * bterm);
} GaussianMapCell;

class GaussianMap {

  public:
  int width; // or columns
  int height; // or rows
  int x_center_cell;
  int y_center_cell;

  

  double cell_width = 0.01;
  GaussianMapCell *cells = NULL;

  GaussianMap(int w, int h, double cw);
  ~GaussianMap();
  void reallocate();

  GaussianMapCell *refAtCell(int x, int y);  
  GaussianMapCell valAtCell(int x, int y);
  GaussianMapCell bilinValAtCell(double x, double y);
  GaussianMapCell bilinValAtMeters(double x, double y);

  int safeAt(int x, int y);
  void metersToCell(double xm, double ym, int * xc, int * yc);
  void metersToCell(double xm, double ym, double * xc, double * yc);
  void cellToMeters(int xc, int yc, double * xm, double * ym);

  void writeToFileStorage(FileStorage& fsvO);
  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);
  void saveToFile(string filename);
  void loadFromFile(string filename);
  
  void writeCells(FileStorage & fsvO);
  void recalculateMusAndSigmas();

  void rgbDiscrepancyMuToMat(Mat& out);
  void rgbMuToMat(Mat& out);
  void rgbSigmaSquaredToMat(Mat& out);
  void rgbCountsToMat(Mat& out);

  void zMuToMat(Mat& out);
  void zSigmaSquaredToMat(Mat& out);
  void zCountsToMat(Mat& out);


  void zeroBox(int _x1, int _y1, int _x2, int _y2);
  void zero();
  void multS(double scalar);
  void addM(shared_ptr<GaussianMap> map);

  shared_ptr<GaussianMap> copyBox(int _x1, int _y1, int _x2, int _y2);
};

typedef enum {
  BACKGROUND = 0,
  PREDICTED = 1,
  SPACE = 2
} sceneObjectType;

sceneObjectType sceneObjectTypeFromString(string str);
string sceneObjectTypeToString(sceneObjectType sot);

class SceneObject {
  public:
  SceneObject(eePose _eep, int _lci, string _ol, sceneObjectType _sot);
  SceneObject();

  // pose within the scene
  eePose scene_pose;
  int labeled_class_index;
  string object_label;
  sceneObjectType sot;

  void writeToFileStorage(FileStorage& fsvO);
  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);
};

class Scene {
  public:
  int width; // or columns
  int height; // or rows
  int x_center_cell;
  int y_center_cell;

  double cell_width = 0.01;

  std::shared_ptr<MachineState> ms;

  Scene(shared_ptr<MachineState> ms, int w, int h, double cw);
  void reallocate();

  eePose background_pose;
  shared_ptr<GaussianMap> background_map;
  shared_ptr<GaussianMap> predicted_map;
  Mat predicted_segmentation;
  shared_ptr<GaussianMap> observed_map;
  shared_ptr<GaussianMap> discrepancy;
  Mat discrepancy_magnitude;
  // transform image to find hot spots
  Mat discrepancy_density;

  vector<shared_ptr<SceneObject> > predicted_objects;

  double score;

  bool isDiscrepantCell(double threshold, int x, int y);
  bool isDiscrepantCellBilin(double threshold, double x, double y);
  bool isDiscrepantMetersBilin(double threshold, double x, double y);
  void composePredictedMap(double threshold);
  void measureDiscrepancy();
  double assignScore();
  double measureScoreRegion(int _x1, int _y1, int _x2, int _y2);

  shared_ptr<Scene> copyPaddedDiscrepancySupport(double threshold, double pad_meters);
  shared_ptr<Scene> copyBox(int _x1, int _y1, int _x2, int _y2);
  int countDiscrepantCells(double threshold, int _x1, int _y1, int _x2, int _y2);

  void proposeRegion();
  // object only makes map better, but must "win" on at least a fraction of its pixels (prior on number of parts)
  void proposeObject();

  void tryToAddObjectToScene();
  void addObjectToPredictedMap();
  void removeObjectFromPredictedMap();

  void removeSpaceObjects();
  void addSpaceObjects();
  void reregisterBackground();
  void reregisterObject(int i);

  int safeAt(int x, int y);
  void metersToCell(double xm, double ym, int * xc, int * yc);
  void metersToCell(double xm, double ym, double * xc, double * yc);
  void cellToMeters(int xc, int yc, double * xm, double * ym);
  void writeToFileStorage(FileStorage& fsvO);
  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);
  void saveToFile(string filename);
  void loadFromFile(string filename);

  void readPredictedObjects(FileNode & fn);
  void writePredictedObjects(FileStorage & fsvO);
};

// transition tables can be instanced for particular settings; transitions may differ in low gravity, high wind, or soft ground, 
//   especially helpful when running experiments to compare two subroutimnes or parameter choices.
// state_labels can be populated by classLabels or can contain statements about objects, too
class TransitionTable {
  public:
  std::vector<string> state_labels;
  std::vector<string> actions;
  std::vector<double> action_probabilities;

  shared_ptr<int> counts;

  shared_ptr<Scene> prescene;
  shared_ptr<Scene> postscene;
  int performed_action;

  void setPrescene(shared_ptr<Scene> s);
  void setPostscene(shared_ptr<Scene> s);
  void setPerformedAction();

  void recordTransitionSceneObject();
  void setStateLabelsFromClassLabels();

  void setActions(std::vector<string> * actions);
  void setActionProbabilities(std::vector<double> * actions);

  void initCounts();

  void writeToFileStorage(FileStorage& fsvO);
  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);
  void saveToFile(string filename);
  void loadFromFile(string filename);
};

#endif /* _EIN_SCENE_H_ */