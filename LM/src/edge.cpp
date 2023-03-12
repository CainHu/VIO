//
// Created by Cain on 2023/2/20.
//

#include <edge.h>
#include <iostream>

namespace vio {
    unsigned long Edge::_global_edge_id = 0;

    Edge::Edge(int residual_dimension, int num_vertices, const std::vector<std::string> &vertices_types) {
        _jacobians.resize(num_vertices);
        _vertices.reserve(num_vertices);

        _residual.resize(residual_dimension, 1);

        _information.resize(residual_dimension, residual_dimension);
        _information.setIdentity();

        _id = _global_edge_id++;

        if (!vertices_types.empty()) {
            _vertices_types = vertices_types;
        }
    }

    double Edge::chi2() {
        // TODO::  we should not Multiply information here, because we have computed Jacobian = sqrt_info * Jacobian
        return _residual.transpose() * _information * _residual;
//    return _residual.transpose() * _residual;   // 当计算 residual 的时候已经乘以了 sqrt_info, 这里不要再乘
    }

    bool Edge::check_valid() {
        if (!_vertices_types.empty()) {
            // check type info
            for (size_t i = 0; i < _vertices.size(); ++i) {
                if (_vertices_types[i] != _vertices[i]->type_info()) {
                    std::cout << "Vertex type does not match, should be " << _vertices_types[i] <<
                         ", but set to " << _vertices[i]->type_info() << std::endl;
                    return false;
                }
            }
        }
        return true;
    }
}

