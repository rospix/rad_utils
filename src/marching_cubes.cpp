#include <radiation_utils/marching_cubes.h>
#include <iostream>

MarchingCube::MarchingCube() {
}

MarchingCube::~MarchingCube() {
}

MarchingCube::MarchingCube(mrs_lib::Cuboid cuboid, std::vector<int> active_vertices) {
  this->cuboid          = cuboid;
  this->active_vertices = active_vertices;
}

int MarchingCube::getTriangulationIndex() {
  int index = 0;
  for (unsigned int i = 0; i < active_vertices.size(); i++) {
    index += std::pow(2, active_vertices[i]);
  }
  return index;
}

std::vector<int> MarchingCube::lookupEdges(int table_index) {
  std::vector<int> edge_indices;
  int*             tmp = lookup_table[table_index];

  for (int i = 0; i < 16; i++) {
    if (tmp[i] >= 0) {
      edge_indices.push_back(tmp[i]);
    }
  }
  return edge_indices;
}

Eigen::Vector3d MarchingCube::getEdgePoint(int edge_index) {
  switch (edge_index) {
    case 0:
      return (cuboid.vertices()[0] + cuboid.vertices()[1]) / 2.0;
    case 1:
      return (cuboid.vertices()[1] + cuboid.vertices()[2]) / 2.0;
    case 2:
      return (cuboid.vertices()[2] + cuboid.vertices()[3]) / 2.0;
    case 3:
      return (cuboid.vertices()[0] + cuboid.vertices()[3]) / 2.0;
    case 4:
      return (cuboid.vertices()[4] + cuboid.vertices()[5]) / 2.0;
    case 5:
      return (cuboid.vertices()[5] + cuboid.vertices()[6]) / 2.0;
    case 6:
      return (cuboid.vertices()[6] + cuboid.vertices()[7]) / 2.0;
    case 7:
      return (cuboid.vertices()[4] + cuboid.vertices()[7]) / 2.0;
    case 8:
      return (cuboid.vertices()[0] + cuboid.vertices()[4]) / 2.0;
    case 9:
      return (cuboid.vertices()[1] + cuboid.vertices()[5]) / 2.0;
    case 10:
      return (cuboid.vertices()[2] + cuboid.vertices()[6]) / 2.0;
    default:
      return (cuboid.vertices()[3] + cuboid.vertices()[7]) / 2.0;
  }
}

std::vector<mrs_lib::Triangle> MarchingCube::getTriangles() {
  int              index        = getTriangulationIndex();
  std::vector<int> edge_indices = lookupEdges(index);

  std::vector<mrs_lib::Triangle> triangles;
  for (unsigned int i = 0; i < edge_indices.size(); i += 3) {
    mrs_lib::Triangle t(getEdgePoint(edge_indices[i]), getEdgePoint(edge_indices[i + 1]), getEdgePoint(edge_indices[i + 2]));
    triangles.push_back(t);
  }
  return triangles;
}
