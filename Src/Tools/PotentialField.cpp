#include "PotentialField.h"


PotentialField::PotentialField(void)
{
}

PotentialField::~PotentialField(void)
{
}

//adds a global, linear (in distance) Potential in point (x,y) with gradient m
//m>0 -> attractive potential, m<0 repulsive potential
void PotentialField::addLinearPotential(const Potential &potential)
{
  Potential newP(potential.position.x(),potential.position.y(),potential.influence);
  linearPotentials.push_back(newP);
}

//adds a local, quadratic potential with effect only in the area of the circle with radius influenceRadius around point. m is a multiplicative constant
void PotentialField::addQuadraticPotential(const QPotential &potential)
{
  QPotential newP(potential.position.x(),potential.position.y(),potential.influenceRadius,potential.influence);
  quadraticPotentials.push_back(newP);
}

void PotentialField::addEllipticalPotential(const EPotential &potential)
{
  EPotential newP(potential.position,potential.influence,potential.left,potential.right);
  ellipticalPotentials.push_back(newP);
}

void PotentialField::addRectangularPotential(const RPotential &potential)
{
  RPotential newP(potential.position,potential.influence,potential.front,potential.back,potential.left,potential.right);
  rectangularPotentials.push_back(newP);
}


Vector2f PotentialField::getGradient(const Vector2f &point)
{

  Vector2f gradient(0,0);

  //linear Potentials

  for(unsigned int i=0; i<linearPotentials.size();i++)
  {
    Potential potential = linearPotentials.at(i);
    float distance = potential.distance(point);
    if(distance!=0)
    {
      gradient.x()+=-potential.influence*(point.x()-potential.position.x())/distance;
      gradient.y()+=-potential.influence*(point.y()-potential.position.y())/distance;
    }
  }


  //quadratic potentials
  for(unsigned int i=0;i<quadraticPotentials.size();i++)
  {
    QPotential qPotential = quadraticPotentials.at(i);
    float distance = qPotential.distance(point);
    distance = std::max(distance,1.f);
    //point in influenceRadius of potential
    if(distance < qPotential.influenceRadius)
    {
      //gradient.x+=qPotential.influence*(1/distance-1/qPotential.influenceRadius)*(1/pow(distance,3.0))*(point.x-qPotential.position.x);
      //gradient.y+=qPotential.influence*(1/distance-1/qPotential.influenceRadius)*(1/pow(distance,3.0))*(point.y-qPotential.position.y);
      gradient.x()+=qPotential.influence*(qPotential.influenceRadius-distance)*(point.x()-qPotential.position.x())/(distance);
      gradient.y()+=qPotential.influence*(qPotential.influenceRadius-distance)*(point.y()-qPotential.position.y())/(distance);

    }
  }

  //ellipticalPotentials

  for(unsigned int i=0;i<ellipticalPotentials.size();i++)
  {
    EPotential ePot = ellipticalPotentials.at(i);
    //distance to mass center
    float distance = ePot.distance(point);
    distance = std::max(distance,1.f);

    //calculate of the intersection between ellipsis and line from center to point
    float alpha = ePot.angleTo(point);
    float influence = ePot.influenceAtAngle(alpha);
    float influenceRadius = ePot.influenceRadiusAtAngle(alpha);


    if(distance < influenceRadius)
    {

      //calculate repulsion: direction is the line from center to point, strength is the distance to central line of the ellipsis

      //gradient.x+=influence*(1/distance-1/influenceRadius)*(1/pow(distance,3.0))*(point.x-ePot.position.x);
      //gradient.y+=influence*(1/distance-1/influenceRadius)*(1/pow(distance,3.0))*(point.y-ePot.position.y);
      gradient.x()+=influence*(influenceRadius-distance)*(point.x()-ePot.position.x())/(distance);
      gradient.y()+=influence*(influenceRadius-distance)*(point.y()-ePot.position.y())/(distance);
    }

  }

  //rectangularPotentials (linear impact)
  for(unsigned int i=0;i<rectangularPotentials.size();i++)
  {
    RPotential rPot = rectangularPotentials.at(i);
    Vector2f direction = point-rPot.position;
    float distance = direction.norm()+0.0001f;
    if(distance<rPot.fitToRectangle(direction).norm())
    {
      gradient.x()+=rPot.influence*(direction.x())/distance;
      gradient.y()+=rPot.influence*(direction.y())/distance;
    }

  }

  return gradient;

}

void PotentialField::clear()
{
  linearPotentials.clear();
  quadraticPotentials.clear();
  ellipticalPotentials.clear();
  rectangularPotentials.clear();
}
