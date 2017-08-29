#include "features3d.h"
#include <stdexcept>
#include <iostream>
#include <pugixml.hpp>
#include "rigidtransformation.h"

using namespace clubster;

Features3D::Features3D(const char *filename)
{
    if (filename == NULL)
        throw std::runtime_error("File pointer is NULL");

    // parser for meshlab picked points file.
    pugi::xml_document doc;
    if (doc.load_file(filename))
    {
        pugi::xml_node picked_points = doc.child("PickedPoints");
        for (pugi::xml_node point = picked_points.child("point"); point; point=point.next_sibling("point"))
        {
            float x = point.attribute("x").as_float();
            float y = point.attribute("y").as_float();
            float z = point.attribute("z").as_float();

            // Add point to features 3D
            Point3D p(x, y, z);
            m_points.push_back(p);
        }
    }
    else
    {
        throw std::runtime_error("Something wrong happened when loading the xml file");
    }
}

void Features3D::transformFeatureSet(const RigidTransformation &rt)
{
    for (size_t i=0; i<m_points.size(); i++)
    {
        Eigen::Vector3d p = m_points[i].getPointAsVec();
        auto tp = rt.transformPoint(p);
        m_points[i].setPoint(tp);
    }
}

Eigen::Vector3d Point3D::getPointAsVec() const
{
    Eigen::Vector3d pvec(m_x, m_y, m_z);
    return pvec;
}

void Point3D::setPoint(Eigen::Vector3d &p)
{
    m_x = p(0);
    m_y = p(1);
    m_z = p(2);
}

