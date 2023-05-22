#include "convex_plane_decomposition/PlaneDecompositionPipeline.h"

#include <opencv2/core/eigen.hpp>

namespace convex_plane_decomposition {

PlaneDecompositionPipeline::PlaneDecompositionPipeline(const Config& config)
    : preprocessing_(config.preprocessingParameters),
      slidingWindowPlaneExtractor_(config.slidingWindowPlaneExtractorParameters, config.ransacPlaneExtractorParameters),
      contourExtraction_(config.contourExtractionParameters),
      postprocessing_(config.postprocessingParameters) {}

void PlaneDecompositionPipeline::update(grid_map::GridMap&& gridMap, const std::string& elevationLayer) {
  // Clear / Overwrite old result
  planarTerrain_.planarRegions.clear();
  planarTerrain_.gridMap = std::move(gridMap);

  //! 1. preprocess(inpaint & denoise & changeResolution)
  preprocessTimer_.startTimer();
  preprocessing_.preprocess(planarTerrain_.gridMap, elevationLayer);
  preprocessTimer_.endTimer();

  //! 2. sliding window plane extractor(runSlidingWindowDetector & runSegmentation & extractPlaneParametersFromLabeledImage(这里应该就有normal vector了吧))
  slidingWindowTimer_.startTimer();
  slidingWindowPlaneExtractor_.runExtraction(planarTerrain_.gridMap, elevationLayer);
  slidingWindowTimer_.endTimer();

  //! 3. contour extraction
  contourExtractionTimer_.startTimer();
  planarTerrain_.planarRegions = contourExtraction_.extractPlanarRegions(slidingWindowPlaneExtractor_.getSegmentedPlanesMap());
  contourExtractionTimer_.endTimer();

  //! used to compute surface normal
  slidingWindowPlaneExtractor_.addSurfaceNormalToMap(planarTerrain_.gridMap, "normal");

  //! 4. postprocess
  postprocessTimer_.startTimer();
  // Add binary map
  const std::string planeClassificationLayer{"plane_classification"};
  planarTerrain_.gridMap.add(planeClassificationLayer);
  auto& traversabilityMask = planarTerrain_.gridMap.get(planeClassificationLayer);
  cv::cv2eigen(slidingWindowPlaneExtractor_.getBinaryLabeledImage(), traversabilityMask);

  postprocessing_.postprocess(planarTerrain_, elevationLayer, planeClassificationLayer);
  postprocessTimer_.endTimer();
}

void PlaneDecompositionPipeline::getSegmentation(grid_map::GridMap::Matrix& segmentation) const {
  cv::cv2eigen(slidingWindowPlaneExtractor_.getSegmentedPlanesMap().labeledImage, segmentation);
}

}  // namespace convex_plane_decomposition