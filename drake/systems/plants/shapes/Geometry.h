#pragma once

#include <string>

#include "drake/common/eigen_types.h"

#include "drake/drakeShapes_export.h"

namespace DrakeShapes {
enum DRAKESHAPES_EXPORT Shape {
  UNKNOWN = 0,
  BOX = 1,
  SPHERE = 2,
  CYLINDER = 3,
  MESH = 4,
  MESH_POINTS = 5,
  CAPSULE = 6
};

std::string ShapeToString(Shape ss);

const double MIN_RADIUS = 1e-7;

class DRAKESHAPES_EXPORT Geometry {
 public:
  Geometry();
  Geometry(const Geometry &other);

  virtual ~Geometry() {}

  virtual Geometry *clone() const;

  Shape getShape() const;

  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const {
    points = Eigen::Matrix3Xd();
  }

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Geometry &);

 protected:
  explicit Geometry(Shape shape);
  void getBoundingBoxPoints(double x_half_width, double y_half_width,
                            double z_half_width,
                            Eigen::Matrix3Xd &points) const;

  Shape shape;
  static const int NUM_BBOX_POINTS;
};

class DRAKESHAPES_EXPORT Sphere : public Geometry {
 public:
  explicit Sphere(double radius);
  virtual ~Sphere() {}
  virtual Sphere *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Sphere &);

  double radius;
  static const int NUM_POINTS;
};

class DRAKESHAPES_EXPORT Box : public Geometry {
 public:
  explicit Box(const Eigen::Vector3d &size);
  virtual ~Box() {}
  virtual Box *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;
  virtual void getTerrainContactPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Box &);

  Eigen::Vector3d size;
};

class DRAKESHAPES_EXPORT Cylinder : public Geometry {
 public:
  Cylinder(double radius, double length);
  virtual ~Cylinder() {}
  virtual Cylinder *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Cylinder &);

  double radius;
  double length;
};

class DRAKESHAPES_EXPORT Capsule : public Geometry {
 public:
  Capsule(double radius, double length);
  virtual ~Capsule() {}
  virtual Capsule *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Capsule &);

  double radius;
  double length;

  static const int NUM_POINTS;
};

class DRAKESHAPES_EXPORT Mesh : public Geometry {
 public:
  explicit Mesh(const std::string &filename);
  Mesh(const std::string &filename, const std::string &resolved_filename);
  virtual ~Mesh() {}
  virtual Mesh *clone() const;
  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const Mesh &);

  Eigen::Vector3d scale;
  std::string filename;
  std::string resolved_filename;
  bool extractMeshVertices(Eigen::Matrix3Xd &vertex_coordinates) const;

  /** Read mesh triangles into the provided array connectivities.

  @param[out] connectivities Each column holds the connectivities for a
  particular triangle in the mesh. Therefore the number of columns equals the
  number of triangles. The i-th column, of size 3, holds the indexes to the
  three vertices of the i-th triangle in the mesh.

  The array `connectivities` is resized to hold the number of triangles in the
  mesh. If successful, `connectivities` will be a matrix with as many columns as
  triangles in the mesh and three rows. Each column corresponds to a triangle
  and contains the indexes to the points loaded with Mesh::extractMeshVertices.
  **/
  bool ReadMeshConnectivities(Eigen::Matrix3Xi& connectivities) const;

  // TODO(amcastro-tri): CREATE ISSUE ON HAVING TO MAKE THIS METHOD CONST!!
  // THIS DOES NOT ALLOW TO LOAD DATA ON THE ACTUAL MESH

  /** Loads triangle mesh from an obj file into the provided vectors of vertices
  and triangles.

  @param[out] vertices Vector of 3D vertices in the mesh.
  @param[out] triangles Vector of indices for each triangle in the mesh.
  The i-th entry of @p triangles holds a 3D vector of integer indices into
  @p vertices corresponding to the vertices forming the i-th triangle.

  On output, `vertices.size()` corresponds to the number of vertices in the mesh
  while `triangles.size()` corresponds to the number of triangles in the mesh.
  **/
  void LoadObjFile(std::vector<Eigen::Vector3d>& vertices,
                   std::vector<Eigen::Vector3i>& triangles) const;

 protected:
  std::string root_dir;
};

class DRAKESHAPES_EXPORT MeshPoints : public Geometry {
 public:
  explicit MeshPoints(const Eigen::Matrix3Xd &points);
  virtual ~MeshPoints() {}
  virtual MeshPoints *clone() const;

  virtual void getPoints(Eigen::Matrix3Xd &points) const;
  virtual void getBoundingBoxPoints(Eigen::Matrix3Xd &points) const;

  /**
   * A toString method for this class.
   */
  friend DRAKESHAPES_EXPORT std::ostream &operator<<(std::ostream &,
                                                     const MeshPoints &);

  Eigen::Matrix3Xd points;
};
}
