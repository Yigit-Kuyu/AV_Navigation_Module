#ifndef _ASTAR2_Taffete_eurecat_
#define _ASTAR2_Taffete_eurecat_

#include <array>
#include <set>
#include <map>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <functional>

namespace AStar
{

struct Coord2D
{
    int16_t x, y;
    Coord2D(): x(-1), y(-1) {}
    Coord2D(int x_, int y_): x(x_), y(y_) {}

    bool operator != (const Coord2D& other) const {
      return !(*this == other);
    }

    bool operator == (const Coord2D& other) const {
      return (x == other.x && y == other.y);
    }

    Coord2D operator + ( const Coord2D& other) const {
      return{ x + other.x, y + other.y };
    }

    Coord2D operator - (const Coord2D& other) const {
      return{ x - other.x, y - other.y };
    }
};


using HeuristicFunction = std::function<uint32_t(Coord2D, Coord2D)>;
using CoordinateList = std::vector<Coord2D>;

struct OpenNode
{
    uint32_t score;
    Coord2D  coord;
    uint8_t  prev_dir; // previous direction
};

// Note: we want the priority_queue to be ordered from smaller to larger
inline bool operator<(const OpenNode& a, const OpenNode& b)
{
  return a.score > b.score;
}

class PathFinder
{

public:
    PathFinder();
    ~PathFinder();

    enum Heuristic
    {
      MANHATTAN, // common heursistic
      EUCLIDEAN, // expensive
      OCTOGONAL, // default
      CUSTOM     // automatically set by setCustomHeuristic()
    };

    ///
    /**
     * @brief setWorldData to be called at least once before findPath
     *
     * @param width            width of the image
     * @param height           height of the image
     * @param data             Row-major ordered map, where an obstacle is represented as a pixel with value 0 (black)
     * @param bytes_per_line   Number of bytes per line (for padded data, eg. images). Default means image is not padded
     * @param color_threshold  threshold used to detect if a grey pixel is an obstacle
     */
    void setWorldData(unsigned width, unsigned height, const uint8_t *data, size_t bytes_per_line=0, uint8_t color_threshold = 120);
    // color_threshold = 20 // orj

    void setHeuristic(Heuristic heuristic);

    void setCustomHeuristic(HeuristicFunction heuristic_);

    /// Function that performs the actual A* computation.
    CoordinateList findPath(Coord2D source_, Coord2D target_, int direction);


    /// Export the resulting solution in a visual way. Useful for debugging.
    void exportPPM(const char* filename, CoordinateList* path);

    struct Cell{
        bool  already_visited;
        uint8_t  path_parent;
        uint32_t cost_G;
        
        Cell(): already_visited(false), cost_G(std::numeric_limits< decltype(cost_G)>::max()) {}
    };

    const Cell& cell(const Coord2D& coordinates_) const
    {
        return _gridmap[coordinates_.y*_world_width + coordinates_.x];
    }

    Cell& cell(const Coord2D& coordinates_)
    {
        return _gridmap[coordinates_.y*_world_width + coordinates_.x];
    }

private:

    Heuristic _heuristic_type;
    HeuristicFunction _heuristic_func;
    uint8_t _obstacle_threshold=0;
    uint32_t _world_width=0;
    uint32_t _world_height=0;
    const uint8_t* _world_data=nullptr;
    size_t _bytes_per_line=0;
    
    std::array<Coord2D,8>  _directions;
    std::array<int,8>      _direction_cost;
    std::vector<std::vector<int>> _direction_next;

    std::priority_queue<OpenNode, std::vector<OpenNode>> _open_set;

    bool detectCollision(const Coord2D& coordinates);

    std::vector<Cell> _gridmap;

    void  clear();
};

class HeuristicImpl
{
public:
    static uint32_t manhattan(const Coord2D& source_, const Coord2D& target_);
    static uint32_t euclidean(const Coord2D& source_, const Coord2D& target_);
    static uint32_t octagonal(const Coord2D& source_, const Coord2D& target_);
};


}

#endif
