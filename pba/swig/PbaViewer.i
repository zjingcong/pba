
%module PbaViewer 
%{
#include "PbaViewer.h"
%}

%include "std_string.i"
%include "std_vector.i"
%template(StringArray) std::vector<std::string>;
%include "PbaViewer.h"
