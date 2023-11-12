//
// Created by Michael on 11/11/2023.
//

//#include "astar_search.h"

# include "iostream"
# include "pybind11/pybind11.h"
# include "pybind11/stl.h"
# include "algorithm"
# include "vector"
# include "utility"
# include "unordered_set"
# include "cstdlib"  // For std::abs with integer types


// renames
namespace py = pybind11;

// constants
std::vector<std::pair<int, int>> SEARCH_DIRECTIONS = {
        {0, -1},
        {0, 1},
        {-1, 0},
        {1, 0}
};


class Node{
    public:
        // vars
        std::shared_ptr<Node> parent;
        std::pair<int, int> position;
        int g_cost;
        int h_cost;
        int f_cost;

        // default constructor
        Node() : parent(nullptr), position({0, 0}), g_cost(0), h_cost(0), f_cost(0) {}

        // constructor with specified params, representing a step on our taken path
        Node(std::shared_ptr<Node> parent,
                 std::pair<int, int> position)
                 : parent(std::move(parent)), position(std::move(position)),
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

};

void make_min_heap(std::vector<Node>& frontier_list){
    std::make_heap(frontier_list.begin(), frontier_list.end(), [](const Node& node_a, const Node& node_b) {
        return node_a.f_cost > node_b.f_cost;
    });
}

bool is_in_maze(const std::pair<int, int>& pos, const std::vector<std::vector<int>>& maze) {
    // Check if the x-coordinate is out of bounds
    if (pos.first < 0 || pos.first >= maze.size()) {
        return false; // x is out of bounds
    }
    // Check if the y-coordinate is out of bounds
    if (pos.second < 0 || pos.second >= maze[pos.first].size()) {
        return false; // y is out of bounds
    }
    return true;
}


std::vector<std::pair<int, int>> compile_path_taken(const Node & current_pos){
    std::vector<std::pair<int, int>> taken_path;
    std::shared_ptr<Node> previous_node = current_pos.parent;

    // go back up through the pointers in parent and collect the positions stop when we reach a node with no parent, ie the start node
    while (previous_node != nullptr){
        taken_path.push_back(previous_node -> position);
        previous_node = previous_node -> parent;
    }
    std::reverse(taken_path.begin(), taken_path.end());
    taken_path.push_back(current_pos.position); // current pos was skipped add it to the end
    return taken_path;
}

Node* find_node_potential_directions(const Node & current_node,
                                    const std::pair<int, int> & direction,
                                    const std::vector<std::vector<int>> & maze){

    std::pair<int, int> new_pos = {current_node.position.first + direction.first,
                                   current_node.position.second + direction.second};

    if (!is_in_maze(new_pos, maze) || maze[new_pos.first][new_pos.second] == 1){
        return nullptr;
    }
    else{
        auto parent_ptr = std::make_shared<Node>(current_node);
        return new Node(parent_ptr, new_pos);
    }

}


// main a* search function
std::vector<std::pair<int, int>> a_star_search(const std::vector<std::vector<int>>& maze,
                                               std::pair<int, int> start_pos,
                                               std::pair<int, int> end_pos){
/*
 * implementation of a* search designed to be called from Python intended args are.
 *
 *      maze: an array of arrays of ints with 1 representing a space that cannot be traveled to
 *      and any other int (ideally 0 representing a space that can be moved to).
 *
 *      start_pos: a pair representing the point to start from on the given maze
 *                 with int 0 being the x-axis and 1 being y.
 *
 *      end_pos: a pair that is the similar to start_pos but represent the intended goal to move towards
 *
 *      return: vector of pairs, representing the order of adjacent coords to travel to reach the goal
 */

    Node start_node;
    Node end_node;

    start_node.position = start_pos;
    end_node.position = end_pos;

    std::vector<Node> frontier_list;
    std::vector<Node> visted_list;

    make_min_heap(frontier_list);
    frontier_list.push_back(start_node);
    std::push_heap(frontier_list.begin(), frontier_list.end());

    int number_of_iterations = 0;
    int max_iterations = (pow(floor(maze.size() / 2), 10.0));

    std::cout << "starting a* search" << std::endl;
    while (!frontier_list.empty()){
        number_of_iterations ++;
        if (number_of_iterations > max_iterations){
            return {}; // return an empty vector
        }

        // move the smallest value to the end and pop it off into a var and remove it from the searchable positions
        std::pop_heap(frontier_list.begin(), frontier_list.end());
        Node current_pos = frontier_list.back();
        frontier_list.pop_back(); // remove the node

        visted_list.push_back(current_pos);

        std::cout << "Current pos is: " << current_pos.position.first << " " << current_pos.position.second << std::endl;

        if (current_pos == end_node){
            return compile_path_taken(current_pos);
        }

        std::vector<Node> children;
        for (std::pair<int, int> search_direction : SEARCH_DIRECTIONS) {
            Node *node_for_direction = find_node_potential_directions(current_pos,
                                                                      search_direction,
                                                                      maze);
            if (!node_for_direction) {
                continue;
            }

            if (std::find(visted_list.begin(), visted_list.end(), *node_for_direction) != visted_list.end()){
                // if we have visited this node before be don't need to calculate its costs
                continue;
            }

            // if this is a node we can consider
            node_for_direction -> g_cost = current_pos.g_cost + 1;

            int x_h_cost = std::abs(node_for_direction -> position.first - end_node.position.first);
            int y_h_cost = std::abs(node_for_direction -> position.second - end_node.position.second);

            node_for_direction -> h_cost = x_h_cost + y_h_cost;
            node_for_direction -> f_cost = node_for_direction -> g_cost + node_for_direction -> h_cost;

            for (const Node& move_candidate : frontier_list){
                if (*node_for_direction == move_candidate && node_for_direction -> g_cost > move_candidate.g_cost){
                    continue;
                }
            }

            // de reference the node and place it in our vector
            children.push_back(*node_for_direction);
        }

        for (const Node& child : children){

            frontier_list.push_back(child);
            std::push_heap(frontier_list.begin(), frontier_list.end());

        }

    }

    return {};
}




// Python bindings, for defining how this file can be accessed via python
PYBIND11_MODULE(astar_search_cpp, m) {
    m.def("a_star_search", &a_star_search, "A function that performs A* search on a maze");
}
