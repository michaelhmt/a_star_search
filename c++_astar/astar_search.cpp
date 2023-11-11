//
// Created by Michael on 11/11/2023.
//

//#include "astar_search.h"
# include "pybind11/pybind11.h"
# include "pybind11/stl.h"
# include "vector"
# include "unordered_set"

// renames
namespace py = pybind11;

class Node{
    public:
        // vars
        std::shared_ptr<Node> parent;
        std::pair<int, int> position;
        double g_cost;
        double h_cost;
        double f_cost;

        //constructor
        Node(std::shared_ptr<Node> parent,
             std::pair<int, int> position)
             : parent(parent), position(position),
             g_cost(0), h_cost(0), f_cost(0) {}

        // methods
        // equality comparison, the most expensive part of the star search
        bool operator == (const Node& other) const{
            return position == other.position;
        }

        // less than operator override
        bool operator < (const Node& other) const{
            return f_cost < other.f_cost;
        }

        // hash function, fow when this class is stored in the heap list
        struct HashFunction {
            size_t operator()(const Node& node) const {
                return std::hash<int>()(node.position.first) ^ std::hash<int>()(node.position.second);
            }
        };
};

// Define a hash function and equality check for std::shared_ptr<Node>
namespace std {
    template<>
    struct hash<shared_ptr<Node>> {
        size_t operator()(const shared_ptr<Node> &node) const {
            return Node::HashFunction()(*node);
        }
    };

    template<>
    struct equal_to<shared_ptr<Node>> {
        bool operator()(const shared_ptr<Node> &lhs, const shared_ptr<Node> &rhs) const {
            return *lhs == *rhs;
        }
    };
}


// Python bindings, for defining how this file can be accessed via python
PYBIND11_MODULE(astar_search_cpp, m) {
    py::class_<Node, std::shared_ptr<Node>>(m, "Node")
            .def(py::init<std::shared_ptr<Node>, std::pair<int, int>>())
            .def_readwrite("parent", &Node::parent)
            .def_readwrite("position", &Node::position)
            .def_readwrite("g_cost", &Node::g_cost)
            .def_readwrite("h_cost", &Node::h_cost)
            .def_readwrite("f_cost", &Node::f_cost)
            .def("__eq__", [](const Node &self, const Node &other) {
                return self == other;
            }, py::is_operator())
            .def("__lt__", [](const Node &self, const Node &other) {
                return self < other;
            }, py::is_operator());
}
