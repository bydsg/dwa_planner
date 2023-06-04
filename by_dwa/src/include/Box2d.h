#ifndef _BOX2D_H_
#define _BOX2D_H_
#include "headfile.h"


class Vec2d {
 public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}
  double x() const { return x_; }
  double y() const { return y_; }
 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

struct point
{
 double  x;
  double y;
};


class Box_2d {
 public:
  Box_2d(const Vec2d &center, const double heading, const double length,
        const double width, const double speed);
  double cos_heading() const { return cos_heading_; }
  double sin_heading() const { return sin_heading_; }  
  double heading() const { return heading_; }  
  double max_x() const { return max_x_; }
  double min_x() const { return min_x_; }
  double max_y() const { return max_y_; }
  double min_y() const { return min_y_; }
  double half_length() const { return half_length_; }
  double half_width() const { return half_width_; }
  double center_x() const { return center_.x(); }
  double center_y() const { return center_.y(); }
  double half_diagonal() const { return std::hypot(length_, width_)/2;}
  double speed() const { return speed_;}
  bool HasOverlap(const Box_2d &box) const;
  double DistanceTo(const Vec2d &point) const;
  double DistanceTo(const Box_2d &box) const;
  std::vector <point> Box2d_corner;
 private:
  Vec2d center_;
  double length_ = 0.0;
  double width_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double heading_ = 0.0;
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;
  double speed_ = 0.0;
  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();
};

Box_2d::Box_2d(const Vec2d &center, const double heading, const double length,
             const double width, const double speed)
    :
     center_(center),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0),
      speed_(speed),
      heading_(heading),
      cos_heading_(cos(heading)),
      sin_heading_(sin(heading)) 
      { float pi=atan(half_width()/(half_length()+0.01));
      point corner1, corner2, corner3, corner4;
     corner1.x=center_x()+ half_diagonal()* cos(heading-pi);
     corner1.y=center_y()+ half_diagonal()* sin(heading- pi );
     Box2d_corner.push_back(corner1); 
     corner2.x=center_x()+ half_diagonal()* cos(heading +pi );
     corner2.y=center_y()+ half_diagonal()* sin(heading+ pi );
     Box2d_corner.push_back(corner2);
     Box2d_corner.push_back(corner2);
     corner3.x=center_x()+ half_diagonal()* cos(heading- pi +M_PI);
     corner3.y=center_y()+ half_diagonal()* sin(heading- pi +M_PI);
     Box2d_corner.push_back(corner3);
      Box2d_corner.push_back(corner3);
     corner4.x=center_x()+ half_diagonal()* cos(heading+pi +M_PI);
     corner4.y=center_y()+ half_diagonal()* sin(heading+ pi +M_PI);
     Box2d_corner.push_back(corner4);
    Box2d_corner.push_back(corner4);
    Box2d_corner.push_back(corner1);
     
     }

double Box_2d::DistanceTo(const Vec2d &point) const {
const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx =
      std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
  const double dy =
      std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return hypot(dx, dy);
}
double Box_2d::DistanceTo(const Box_2d &box) const {
  return  sqrt( pow(box.center_.x() - center_.x(),2)+pow(box.center_.y() - center_.y(),2)) ;
}

bool Box_2d::HasOverlap(const Box_2d &box) const {
  if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() ||
      box.min_y() > max_y()) {
    return false;
  }

  const double shift_x = box.center_x() - center_.x();
  const double shift_y = box.center_y() - center_.y();
  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  const double dx3 = box.cos_heading() * box.half_length();
  const double dy3 = box.sin_heading() * box.half_length();
  const double dx4 = box.sin_heading() * box.half_width();
  const double dy4 = -box.cos_heading() * box.half_width();

  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
             std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                 std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                 half_length_ &&
         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
             std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                 std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                 half_width_ &&
         std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
             std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
                 std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
                 box.half_length() &&
         std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
             std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
                 std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
                 box.half_width();
}

#endif