#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <mrs_lib/attitude_converter.h>


namespace mapping
{

class GridMap {
public:
  /**
   * @brief Create a new GridMap
   *
   * @param width - map width in meters
   * @param height - map height in meters
   * @param resolution - cell size in meters
   * @param origin - position of the map origin in the parent frame
   * @param rotation - rotation relative to the parent frame
   * @param parent_frame - name of the parent frame
   */
  GridMap(double width, double height, double resolution, Eigen::Vector2d origin, double rotation, std::string parent_frame);
  ~GridMap();

  /**
   * @brief width (x-coordinate) of the grid in meters
   *
   * @return width in meters
   */
  double getWidth();


  /**
   * @brief height (y-coordinate) of the grid in meters
   *
   * @return height in meters
   */
  double getHeight();

  /**
   * @brief get number of rows
   *
   * @return
   */
  int getRows();

  /**
   * @brief get number of columns
   *
   * @return
   */
  int getColumns();

  /**
   * @brief get number of cells
   *
   * @return
   */
  size_t getNumCells();

  /**
   * @brief single cell value
   *
   * @param index of the cell (column + row * num_columns), rows grow in the direction of X-axis, columns in Y-axis
   *
   * @return cell value
   */
  double getValue(int index);


  /**
   * @brief single cell value
   *
   * @param row
   * @param col
   *
   * @return cell value
   */
  double getValue(int row, int col);

  /**
   * @brief single cell value
   *
   * @param coords row and column indices
   *
   * @return cell value
   */
  double getValue(Eigen::Vector2i coords);

  /**
   * @brief Construct nav_msgs::occupancyGrid representation of the grid data. Content is normalized to fit into the <0, 254> range
   *
   * @return nav_msgs::occupancyGrid representation
   */
  nav_msgs::OccupancyGrid buildOccupancyGrid();

  /**
   * @brief set value of a specific cell
   *
   * @param value new cell value
   * @param index cell index
   */
  void setCellValue(double value, int index);

  /**
   * @brief set value of a specific cell
   *
   * @param value new cell value
   * @param row (starts at origin, grows with X-axis)
   * @param col (starts at origin, grows with Y-axis)
   */
  void setCellValue(double value, int row, int col);

  /**
   * @brief set value of a specific cell
   *
   * @param value new cell value
   * @param coords row and column indices
   */
  void setCellValue(double value, Eigen::Vector2i coords);

private:
  std::string     parent_frame;
  Eigen::Vector2d size;
  Eigen::Vector2d origin;

  double resolution;  // [m/cell]
  double width;       // [m]
  double height;      // [m]
  double rotation;    // [rad]

  int rows;
  int columns;

  std::vector<double> cells;

  size_t getIndex(int index);
  size_t getIndex(int row, int col);
  size_t getIndex(Eigen::Vector2i coords);
};


}  // namespace mapping
