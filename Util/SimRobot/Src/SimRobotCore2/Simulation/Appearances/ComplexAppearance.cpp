/**
* @file Simulation/Appearances/ComplexAppearance.cpp
* Implementation of class ComplexAppearance
* @author Colin Graf
*/

#include <cmath>
#include "Platform/OpenGL.h"

#include "Simulation/Appearances/ComplexAppearance.h"
#include "Tools/Texture.h"
#include "Platform/Assert.h"

void ComplexAppearance::PrimitiveGroup::addParent(Element& element)
{
  ComplexAppearance* complexAppearance = dynamic_cast<ComplexAppearance*>(&element);
  complexAppearance->primitiveGroups.push_back(this);
}

void ComplexAppearance::Vertices::addParent(Element& element)
{
  ComplexAppearance* complexAppearance = dynamic_cast<ComplexAppearance*>(&element);
  ASSERT(!complexAppearance->vertices);
  complexAppearance->vertices = this;
}

void ComplexAppearance::Normals::addParent(Element& element)
{
  ComplexAppearance* complexAppearance = dynamic_cast<ComplexAppearance*>(&element);
  ASSERT(!complexAppearance->normals);
  complexAppearance->normals = this;
  complexAppearance->normalsDefined = true;
}

void ComplexAppearance::TexCoords::addParent(Element& element)
{
  ComplexAppearance* complexAppearance = dynamic_cast<ComplexAppearance*>(&element);
  ASSERT(!complexAppearance->texCoords);
  complexAppearance->texCoords = this;
}

void ComplexAppearance::createGraphics()
{
  ASSERT(vertices);

  if (initializedContexts == 0)
  {
    size_t verticesSize = vertices->vertices.size();
    if (verticesSize > 0 && !normalsDefined)
    {
      const Vertex* vertexLibrary = &vertices->vertices[0];
      normals = new Normals();
      normals->normals.resize(verticesSize);
      Normal* vertexNormals = &normals->normals[0];

      for (std::list<PrimitiveGroup*>::const_iterator iter = primitiveGroups.begin(), end = primitiveGroups.end(); iter != end; ++iter)
      {
        PrimitiveGroup& primitiveGroup = *(*iter);
        ASSERT(primitiveGroup.mode == GL_TRIANGLES || primitiveGroup.mode == GL_QUADS);
        ASSERT(primitiveGroup.vertices.size() % (primitiveGroup.mode == GL_TRIANGLES ? 3 : 4) == 0);
        for (std::list<unsigned int>::iterator vertIter = primitiveGroup.vertices.begin(), vertEnd = primitiveGroup.vertices.end(); vertIter != vertEnd;)
        {
          unsigned int i1 = *vertIter;
          if (i1 >= verticesSize)
            *vertIter = i1 = 0; // this is bullshit, but better than crashing because of incorrect input files
          ++vertIter;
          unsigned int i2 = *vertIter;
          if (i2 >= verticesSize)
            *vertIter = i2 = 0; // this is bullshit, but better than crashing because of incorrect input files
          ++vertIter;
          unsigned int i3 = *vertIter;
          if (i3 >= verticesSize)
            *vertIter = i3 = 0; // this is bullshit, but better than crashing because of incorrect input files
          ++vertIter;
          unsigned int i4 = 0;
          if (primitiveGroup.mode == GL_QUADS)
          {
            i4 = *vertIter;
            if (i4 >= verticesSize)
              *vertIter = i4 = 0; // this is bullshit, but better than crashing because of incorrect input files
            ++vertIter;
          }

          const Vertex& p1 = vertexLibrary[i1];
          const Vertex& p2 = vertexLibrary[i2];
          const Vertex& p3 = vertexLibrary[i3];

          Vertex u(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
          Vertex v(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
          Normal n(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x, 1);
          float len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
          len = len == 0 ? 1.f : 1.f / len;
          n.x *= len;
          n.y *= len;
          n.z *= len;

          vertexNormals[i1] += n;
          vertexNormals[i2] += n;
          vertexNormals[i3] += n;
          if (primitiveGroup.mode == GL_QUADS)
            vertexNormals[i4] += n;
        }
      }

      for (Normal *i = vertexNormals, *end = vertexNormals + verticesSize; i < end; ++i)
        if (i->length)
        {
          const float mult = 1.f / float(i->length);
          i->x *= mult;
          i->y *= mult;
          i->z *= mult;
        }
    }
  }

  Appearance::createGraphics();
}

void ComplexAppearance::assembleAppearances(SurfaceColor color) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  size_t verticesSize = vertices->vertices.size();
  if (verticesSize > 0)
  {
    surface->set(color, !texCoords);

    const Vertex* vertexLibrary = &vertices->vertices[0];
    const Normal* vertexNormals = &normals->normals[0];

    for (std::list<PrimitiveGroup*>::const_iterator iter = primitiveGroups.begin(), end = primitiveGroups.end(); iter != end; ++iter)
    {
      const PrimitiveGroup& primitiveGroup = *(*iter);
      glBegin(primitiveGroup.mode);
      for (std::list<unsigned int>::const_iterator vertIter = primitiveGroup.vertices.begin(), vertEnd = primitiveGroup.vertices.end(); vertIter != vertEnd; ++vertIter)
      {
        const unsigned int i = *vertIter;
        if (texCoords && i < texCoords->coords.size())
          glTexCoord2fv(&texCoords->coords[i].x);
        if (normalsDefined)
        {
          if (++vertIter != vertEnd)
            glNormal3fv(&vertexNormals[*vertIter].x);
          else
            break;
        }
        else
          glNormal3fv(&vertexNormals[i].x);
        glVertex3fv(&vertexLibrary[i].x);
      }
      glEnd();
    }

    surface->unset(!texCoords);
  }

  GraphicalObject::assembleAppearances(color);
  glPopMatrix();
}
