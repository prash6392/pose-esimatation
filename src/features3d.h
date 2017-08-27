#ifndef FEATURES3D_H
#define FEATURES3D_H

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace clubster
{
    class RigidTransformation;
    class Point3D
    {
    public:
        Point3D() :
            m_x(0),
            m_y(0),
            m_z(0)
        {}
        Point3D(const Point3D& other) :
            m_x(other.m_x),
            m_y(other.m_y),
            m_z(other.m_z)
        {}
        Point3D(const float & x, const float & y, const float & z) :
            m_x(x),
            m_y(y),
            m_z(z)
        {}
        Point3D(const Eigen::Vector3d &v):
            m_x(v(0)),
            m_y(v(1)),
            m_z(v(2))
        {}
        float & getX() {return m_x; }
        const float & getX() const {return m_x;}
        float & getY() {return m_y; }
        const float & getY() const {return m_y;}
        float & getZ() {return m_z; }
        const float & getZ() const {return m_z;}
        void setPoint(Eigen::Vector3d & p);
        Eigen::Vector3d getPointAsVec() const;
        friend std::ostream & operator<<(std::ostream & os, const Point3D& pt)
        {
            os << "x: " << pt.m_x << "\ty: " << pt.m_y << "\tz: " << pt.m_z << std::endl;
            return os;
        }
    private:
        float m_x;
        float m_y;
        float m_z;
    };

    class Features3D
    {
    public:
        Features3D(const char * filename);
        Features3D(const Features3D & other) :
            m_points(other.m_points)
        {}
        void addPoint(Point3D & point);
        const Point3D & getPoint(int i);
        size_t size() const {return m_points.size();}
        void transformFeatureSet(const RigidTransformation & rt);
        friend std::ostream & operator<<(std::ostream & os, const Features3D & feats)
        {
            for (size_t i=0; i<feats.m_points.size(); i++)
            {
                os << "index: " << i << "\t" << feats.m_points[i] << std::endl;
            }
            return os;
        }
        const Point3D & operator[](const int index) const
        {
            if ((size_t)index <m_points.size())
                return m_points[index];
            else
                throw std::runtime_error("Index exceeds dimension of the feature.");
        }

    private:
        std::vector<Point3D> m_points;
    };



} // end - namespace clubster

#endif // FEATURES3D_H
