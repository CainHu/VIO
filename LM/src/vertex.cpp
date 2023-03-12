//
// Created by Cain on 2023/2/20.
//

#include <vertex.h>

namespace vio {
    unsigned long Vertex::_global_vertex_id = 0;

    Vertex::Vertex(int num_dimension, int local_dimension) {
        _parameters.resize(num_dimension, 1);
        _local_dimension = local_dimension > 0 ? local_dimension : num_dimension;
        _id = _global_vertex_id++;
    }

}

