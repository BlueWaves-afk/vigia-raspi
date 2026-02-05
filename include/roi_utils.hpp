#pragma once

#ifdef VIGIA_DEBUG_ROI
#include <iostream>
#endif

#include <opencv2/core.hpp>

namespace vigia {

inline cv::Rect clampROIToMat(const cv::Rect& roi, const cv::Mat& mat) {
    if (mat.empty())
        return {};

    const cv::Rect imageBounds(0, 0, mat.cols, mat.rows);
    cv::Rect clamped = roi & imageBounds;

    if (clamped.width <= 0 || clamped.height <= 0) {
#ifdef VIGIA_DEBUG_ROI
        std::clog << "[ROI] Discarded invalid region x=" << roi.x
                   << " y=" << roi.y
                   << " w=" << roi.width
                   << " h=" << roi.height
                   << " for mat " << mat.cols << "x" << mat.rows << '\n';
#endif
        return {};
    }

    if (clamped == roi)
        return clamped;

#ifdef VIGIA_DEBUG_ROI
    std::clog << "[ROI] Clamped region from x=" << roi.x
               << " y=" << roi.y
               << " w=" << roi.width
               << " h=" << roi.height
               << " to x=" << clamped.x
               << " y=" << clamped.y
               << " w=" << clamped.width
               << " h=" << clamped.height
               << " for mat " << mat.cols << "x" << mat.rows << '\n';
#endif
    return clamped;
}

} // namespace vigia
