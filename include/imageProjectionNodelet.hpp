#pragma once

#include <nodelet/nodelet.h>
#include "imageProjection.hpp"

namespace lio_sam
{
class ImageProjectionNodelet : public nodelet::Nodelet
{
  std::unique_ptr<ImageProjection> m_image_projection;

public:
  virtual void onInit() override;
};
}
