
%module PbaThings
%{
#include "TriangleCollision.h"
#include "BoidFlocking.h"
#include "RBDThing.h"
#include "ClothThing.h"
%}

%include "std_string.i"
%include "std_vector.i"
%template(StringArray) std::vector<std::string>;
%include "TriangleCollision.h"
%include "BoidFlocking.h"
%include "RBDThing.h"
%include "ClothThing.h"
