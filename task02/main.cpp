#include <filesystem>
// #include <experimental/filesystem> // uncomment here if the <filesystem> cannot be included above
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Eigen/Core"
//
#include "parse_svg.h"

/***
 * signed area of a triangle connecting points (p0, p1, p2) in counter-clockwise order.
 * @param p0 1st point xy-coordinate
 * @param p1 2nd point xy-coordinate
 * @param p2 3rd point xy-coordinate
 * @return signed area (float)
 */
float area(
    const Eigen::Vector2f &p0,
    const Eigen::Vector2f &p1,
    const Eigen::Vector2f &p2) {
  const auto v01 = p1 - p0;
  const auto v02 = p2 - p0;
  // return 0.5f * (v01[0] * v02[1] - v01[1] * v02[0]); // right handed coordinate
  return 0.5f * (v01[1] * v02[0] - v01[0] * v02[1]); // left-handed coordinate (because pixel y-coordinate is going down)
}


/***
 * compute number of intersection of a ray against a line segment
 * @param org ray origin
 * @param dir ray direction (unit normal)
 * @param ps one of the two end points
 * @param pe the other end point
 * @return number of intersection
 */
int number_of_intersection_ray_against_edge(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pe) {
  auto a = area(org, org + dir, ps);
  auto b = area(org, pe, org + dir);
  auto c = area(org, ps, pe);
  auto d = area(dir+ps, ps, pe);
  if (a * b > 0.f && d * c < 0.f) { return 1; }
  return 0;
  // the following code was a bug
  //auto d = area(org + dir, ps, pe);
  //if (a * b > 0.f && d * c > 0.f && fabs(d) > fabs(c)) { return 1; }
}

float calc_theta(
  const Eigen::Vector2f &p0,
  const Eigen::Vector2f &p1)
{
  auto p0x = p0[0];
  auto p0y = p0[1];
  auto p1x = p1[0];
  auto p1y = p1[1];

  float p0_size = std::sqrt(p0x * p0x + p0y * p0y);
  float p1_size = std::sqrt(p1x * p1x + p1y * p1y);
  float dot = p0x * p1x + p0y * p1y;
  float cos = dot / (p0_size * p1_size);
  float cross = p0y * p1x - p0x * p1y;
  float sin = cross / (p0_size * p1_size);
  float theta = std::atan2(sin, cos);

  return theta;
}

bool is_in_triangle(
  const Eigen::Vector2f &ps,
  const Eigen::Vector2f &pc,
  const Eigen::Vector2f &pe,
  const Eigen::Vector2f &q)
{
  float winding_number = 0;
  winding_number += calc_theta(ps - q, pc - q);
  winding_number += calc_theta(pc - q, pe - q);
  winding_number += calc_theta(pe - q, ps - q);
  const double PI = acos(-1);
  winding_number /= 2 * PI;
  const int int_winding_number = int(std::round(winding_number));
  if (int_winding_number == 1)
    // std::cout << "flag4" << std::endl;
    return true;
  return false;
}

/***
 *
 * @param org ray origin
 * @param dir ray direction (unit vector)
 * @param ps one of the two end points
 * @param pc control point
 * @param pe the other end point
 * @return the number of intersections
 */
int number_of_intersection_ray_against_quadratic_bezier(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pc,
    const Eigen::Vector2f &pe) {
  // comment out below to do the assignment
  // return number_of_intersection_ray_against_edge(org, dir, ps, pe);
  // write some code below to find the intersection between ray and the quadratic
  

  // express ray as (Ax + By + C = 0)
  const auto a = org;
  const auto b = org + dir;
  const double A = b[1] - a[1];
  const double B = a[0] - b[0];
  const double C = - b[1] * a[0] + b[0] * a[1];
  
  // tの2次方程式の係数
  const double quad_A = (ps[0] - 2 * pc[0] + pe[0]) * A + (ps[1] - 2 * pc[1] + pe[1]) * B;
  const double quad_B = 2 * (-ps[0] + pc[0]) * A + 2 * (-ps[1] + pc[1]) * B;
  const double quad_C = ps[0] * A + ps[1] * B + C;

  // 判別式を解く
  const auto discriminant = quad_B * quad_B - 4 * quad_A * quad_C;
  if (discriminant < 0)
    return 0;
  const auto t0 = (-quad_B + sqrt(quad_B * quad_B - 4 * quad_A * quad_C)) / (2 * quad_A);
  const auto t1 = (-quad_B - sqrt(quad_B * quad_B - 4 * quad_A * quad_C)) / (2 * quad_A);
  // 0 < t < 1
  if ((!isfinite(t0) || t0 <= 0 || 1 <= t0) && (!isfinite(t1) || t1 <= 0 || 1 <= t1))
    return 0;
  const auto t = (!isfinite(t0) || t0 <= 0 || 1 <= t0) ? t1 : t0;
  // qがps, pc, peの三角形の内部に存在するか s < 0対策
  const auto q = (1 - t) * (1 - t) * ps + 2 * t * (1 - t) * pc + t * t * pe;
  if (!is_in_triangle(ps, pc, pe, q)){
    // std::cout << "not in triangle" << std::endl; // for debug
    return 0;
  }
  return 1;
}

int main() {
  const auto input_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / ".." / "asset" / "r.svg";
  const auto [width, height, shape] = acg::svg_get_image_size_and_shape(input_file_path);
  if (width == 0) { // something went wrong in loading the function
    std::cout << "file open failure" << std::endl;
    abort();
  }
  const std::vector<std::string> outline_path = acg::svg_outline_path_from_shape(shape);
  const std::vector<std::vector<acg::Edge>> loops = acg::svg_loops_from_outline_path(outline_path);
  //
  std::vector<unsigned char> img_data(width * height, 255); // grayscale image initialized white
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto org = Eigen::Vector2f(iw + 0.5, ih + 0.5); // pixel center
      const auto dir = Eigen::Vector2f(60., 20.); // search direction
      int count_cross = 0;
      for (const auto &loop: loops) { // loop over loop (letter R have internal/external loops)
        for (const auto &edge: loop) { // loop over edge in the loop
          if (edge.is_bezier) { // in case the edge is a quadratic Bézier
            count_cross += number_of_intersection_ray_against_quadratic_bezier(
                org, dir,
                edge.ps, edge.pc, edge.pe);
          } else { // in case the edge is a line segment
            count_cross += number_of_intersection_ray_against_edge(
                org, dir,
                edge.ps, edge.pe);
          }
        }
      }
      if (count_cross % 2 == 1) { // Jordan's curve theory
        img_data[ih * width + iw] = 0; // paint black if it is inside
      }
    }
  }
  const auto output_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png";
  stbi_write_png(output_file_path.string().c_str(), width, height, 1, img_data.data(), width);
}
