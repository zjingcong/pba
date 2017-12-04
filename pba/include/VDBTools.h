//
// Created by jingcoz on 12/3/17.
//

#ifndef PBA_VDBTOOLS_H
#define PBA_VDBTOOLS_H

# include <openvdb/openvdb.h>
# include "Types.h"
# include <iostream>

template < typename T >
typename openvdb::Grid<T>::Ptr readVDBGrid(std::string vdbFilePath, std::string gridName)
{
    std::cout << "Load openVDB grid " << gridName << " from .vdb file " << vdbFilePath << std::endl;
    openvdb::initialize();
    openvdb::io::File fileR(vdbFilePath);
    fileR.open();
    openvdb::GridBase::Ptr base;
    for (openvdb::io::File::NameIterator iter = fileR.beginName(); iter != fileR.endName(); ++iter)
    {
        std::cout << "grid_name: " << iter.gridName() << std::endl;
        if (iter.gridName() == gridName)    {base = fileR.readGrid(iter.gridName());}
    }
    fileR.close();
    openvdb::uninitialize();
    std::cout << "Generate openVDB grid..." << std::endl;
    typename openvdb::Grid<T>::Ptr grid = openvdb::gridPtrCast<typename openvdb::Grid<T>>(base);
    std::cout << "Generate grid " << gridName << " success." << std::endl;

    return grid;
}


#endif //PBA_VDBTOOLS_H
